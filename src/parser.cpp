#include "parser.h"
#include <string>
#include <sstream>
#include <fstream>
#include <filesystem>
#include "helper.h"
#include <algorithm>
#include <array>

char *parser::get_json_content(const char *fileName)
{
	FILE *fp = fopen(fileName, "rb");
	if (!fp)
		return NULL;

	fseek(fp, 0, SEEK_END);
	long size_l = ftell(fp);
	rewind(fp);

	if (size_l < 0)
	{
		fclose(fp);
		return NULL;
	}
	size_t size = (size_t)size_l;

	char *file = (char *)malloc(size + 1);
	if (!file)
	{
		fclose(fp);
		return NULL;
	}

	size_t read = fread(file, 1, size, fp);
	fclose(fp);

	file[read] = '\0';

	return file;
}

parser::parser(const char *file_name)
{
	json_content = get_json_content(file_name);
	d.Parse(json_content);
	if (d.HasParseError())
	{
		my_printf("RapidJSON parse error at offset %zu: %u\n",
				  d.GetErrorOffset(), d.GetParseError());
	}
	this->file_name = std::string(file_name);
}

void processTrans(const rapidjson::Value &obj, simd_mat4 &calculator_m, transformations *t, mat4 &model)
{
	if (obj.HasMember("Transformations"))
	{
		std::string transformations = obj["Transformations"].GetString();

		std::istringstream iss(transformations);
		std::vector<std::string> result;
		std::string word;

		while (iss >> word)
		{
			result.push_back(word);
		}

		for (auto it = result.begin(); it != result.end(); ++it)
		{
			const std::string &i = *it;
			int index = std::stoi(i.substr(1));
			if (i.starts_with("t"))
			{
				calculator_m.mult(t->translations[index - 1], model, model);
			}
			else if (i.starts_with("r"))
			{
				calculator_m.mult(t->rotations[index - 1], model, model);
			}
			else if (i.starts_with("s"))
			{
				calculator_m.mult(t->scales[index - 1], model, model);
			}
		}
	}
};

std::vector<image *> *parser::get_images()
{
	auto *images = new std::vector<image *>;

	if (!d.HasMember("Scene") || !d["Scene"].HasMember("Textures"))
		return images;

	const auto &texturesNode = d["Scene"]["Textures"];
	if (!texturesNode.HasMember("Images") || !texturesNode["Images"].HasMember("Image"))
		return images;

	const auto &imageNode = texturesNode["Images"]["Image"];

	auto processImage = [&](const rapidjson::Value &img)
	{
		if (!img.HasMember("_id") || !img.HasMember("_data"))
			return;

		int id = std::stoi(img["_id"].GetString());
		std::string relativePath = img["_data"].GetString();

		std::filesystem::path baseDir = std::filesystem::path(this->file_name).parent_path();
		std::filesystem::path fullPath = baseDir / relativePath;

		images->push_back(new image(fullPath.string().c_str(), id));
	};

	if (imageNode.IsArray())
	{
		for (const auto &img : imageNode.GetArray())
		{
			processImage(img);
		}
	}
	else if (imageNode.IsObject())
	{
		processImage(imageNode);
	}

	return images;
}

std::vector<texture *> *parser::get_textures(std::vector<image *> *images)
{
	auto *textures = new std::vector<texture *>;

	if (!d.HasMember("Scene") || !d["Scene"].HasMember("Textures"))
		return textures;

	const auto &texturesNode = d["Scene"]["Textures"];
	if (!texturesNode.HasMember("TextureMap"))
		return textures;

	const auto &texNode = texturesNode["TextureMap"];

	auto processTexture = [&](const rapidjson::Value &tex)
	{
		if (!tex.HasMember("_id") || !tex.HasMember("ImageId"))
			return;

		int id = std::stoi(tex["_id"].GetString());
		int imageId = std::stoi(tex["ImageId"].GetString());

		image *linkedImage = nullptr;
		if (images)
		{
			for (auto *img : *images)
			{
				if (img->id == imageId)
				{
					linkedImage = img;
					break;
				}
			}
		}

		if (!linkedImage)
		{
			my_printf("Warning: Texture %d refers to missing ImageId %d\n", id, imageId);
			return;
		}

		DecalMode mode = replace_kd;
		if (tex.HasMember("DecalMode"))
		{
			std::string modeStr = tex["DecalMode"].GetString();
			if (modeStr == "replace_kd")
				mode = replace_kd;
			else if (modeStr == "blend_kd")
				mode = blend_kd;
			else if (modeStr == "replace_ks")
				mode = replace_ks;
			else if (modeStr == "replace_background")
				mode = replace_background;
			else if (modeStr == "replace_normal")
				mode = replace_normal;
			else if (modeStr == "bump_normal")
				mode = bump_normal;
			else if (modeStr == "replace_all")
				mode = replace_all;
		}

		Interpolation interp = bilinear;
		if (tex.HasMember("Interpolation"))
		{
			std::string interpStr = tex["Interpolation"].GetString();
			if (interpStr == "nearest")
				interp = nearest;
			else if (interpStr == "bilinear")
				interp = bilinear;
		}

		texture *t = new texture(linkedImage, id, mode, interp);

		textures->push_back(t);

		if (tex.HasMember("BumpFactor"))
		{
			std::string bump = tex["BumpFactor"].GetString();
			t->BumpFactor = std::stoi(bump);
		}
	};

	if (texNode.IsArray())
	{
		for (const auto &tex : texNode.GetArray())
		{
			processTexture(tex);
		}
	}
	else if (texNode.IsObject())
	{
		processTexture(texNode);
	}

	return textures;
}

std::vector<camera> *parser::get_camera(simd_vec3 &calculator, simd_mat4 &calculator_m, transformations *t)
{
	std::vector<camera> *res = new std::vector<camera>;

	const rapidjson::Value &cameras = d["Scene"]["Cameras"];

	if (!cameras.HasMember("Camera"))
		return res;

	const rapidjson::Value &camNode = cameras["Camera"];

	auto parseCamera = [&](const rapidjson::Value &cam)
	{
		std::string tmp;

		std::string type = "pinhole";
		if (cam.HasMember("_type"))
			type = cam["_type"].GetString();

		mat4 model;

		processTrans(cam, calculator_m, t, model);

		if (type == "lookAt")
		{
			float position_x, position_y, position_z;
			float gaze_x, gaze_y, gaze_z;
			float up_x, up_y, up_z;
			float neardistance;
			float fovY;
			int NumSamples;
			float FocusDistance, ApertureSize;
			int resx, resy;

			tmp = cam["Position"].GetString();
			sscanf(tmp.c_str(), "%f %f %f", &position_x, &position_y, &position_z);

			tmp = cam["GazePoint"].GetString();
			sscanf(tmp.c_str(), "%f %f %f", &gaze_x, &gaze_y, &gaze_z);

			tmp = cam["Up"].GetString();
			sscanf(tmp.c_str(), "%f %f %f", &up_x, &up_y, &up_z);

			tmp = cam["FovY"].GetString();
			sscanf(tmp.c_str(), "%f", &fovY);

			tmp = cam["NearDistance"].GetString();
			sscanf(tmp.c_str(), "%f", &neardistance);

			tmp = cam["ImageResolution"].GetString();
			sscanf(tmp.c_str(), "%d %d", &resx, &resy);

			if (cam.HasMember("NumSamples"))
			{
				tmp = cam["NumSamples"].GetString();
				sscanf(tmp.c_str(), "%d", &NumSamples);
			}
			else
			{
				NumSamples = 1;
			}

			if (cam.HasMember("FocusDistance"))
			{
				tmp = cam["FocusDistance"].GetString();
				sscanf(tmp.c_str(), "%f", &FocusDistance);
			}
			else
			{
				FocusDistance = 0;
			}

			if (cam.HasMember("ApertureSize"))
			{
				tmp = cam["ApertureSize"].GetString();
				sscanf(tmp.c_str(), "%f", &ApertureSize);
			}
			else
			{
				ApertureSize = 0;
			}

			tmp = cam["ImageName"].GetString();

			res->emplace_back(
				calculator, calculator_m, position_x, position_y, position_z,
				gaze_x, gaze_y, gaze_z, up_x, up_y, up_z,
				neardistance, fovY, resx, resy, NumSamples, FocusDistance, ApertureSize, tmp, model);
		}
		else
		{
			float position_x, position_y, position_z;
			float gaze_x, gaze_y, gaze_z;
			float up_x, up_y, up_z;
			float nearp_left, nearp_right, nearp_bottom, nearp_top;
			float neardistance;
			int NumSamples;
			float FocusDistance, ApertureSize;
			int resx, resy;

			tmp = cam["Position"].GetString();
			sscanf(tmp.c_str(), "%f %f %f", &position_x, &position_y, &position_z);

			tmp = cam["Gaze"].GetString();
			sscanf(tmp.c_str(), "%f %f %f", &gaze_x, &gaze_y, &gaze_z);

			tmp = cam["Up"].GetString();
			sscanf(tmp.c_str(), "%f %f %f", &up_x, &up_y, &up_z);

			tmp = cam["NearPlane"].GetString();
			sscanf(tmp.c_str(), "%f %f %f %f", &nearp_left, &nearp_right, &nearp_bottom, &nearp_top);

			tmp = cam["NearDistance"].GetString();
			sscanf(tmp.c_str(), "%f", &neardistance);

			tmp = cam["ImageResolution"].GetString();
			sscanf(tmp.c_str(), "%d %d", &resx, &resy);

			if (cam.HasMember("NumSamples"))
			{
				tmp = cam["NumSamples"].GetString();
				sscanf(tmp.c_str(), "%d", &NumSamples);
			}
			else
			{
				NumSamples = 1;
			}

			if (cam.HasMember("FocusDistance"))
			{
				tmp = cam["FocusDistance"].GetString();
				sscanf(tmp.c_str(), "%f", &FocusDistance);
			}
			else
			{
				FocusDistance = 0;
			}

			if (cam.HasMember("ApertureSize"))
			{
				tmp = cam["ApertureSize"].GetString();
				sscanf(tmp.c_str(), "%f", &ApertureSize);
			}
			else
			{
				ApertureSize = 0;
			}

			tmp = cam["ImageName"].GetString();

			res->emplace_back(calculator, calculator_m, position_x, position_y, position_z,
							  gaze_x, gaze_y, gaze_z, up_x, up_y, up_z,
							  neardistance, nearp_left, nearp_right, nearp_bottom, nearp_top,
							  resx, resy, NumSamples, FocusDistance, ApertureSize, tmp, model);
		}
	};

	if (camNode.IsObject())
		parseCamera(camNode);
	else if (camNode.IsArray())
		for (auto &c : camNode.GetArray())
			parseCamera(c);

	return res;
}

std::vector<vec3> *parser::get_vertices()
{
	std::vector<vec3> *res = new std::vector<vec3>;

	if (!d["Scene"].HasMember("VertexData"))
	{
		return res;
	}

	std::istringstream iss(d["Scene"]["VertexData"]["_data"].GetString());
	float x, y, z;

	while (iss >> x >> y >> z)
	{
		res->emplace_back(x, y, z);
	}

	return res;
}

std::vector<float> *parser::get_uvs()
{
	std::vector<float> *res = new std::vector<float>;

	if (!d["Scene"].HasMember("TexCoordData"))
	{
		return res;
	}

	std::istringstream iss(d["Scene"]["TexCoordData"]["_data"].GetString());
	float x;

	while (iss >> x)
	{
		res->push_back(x);
	}

	return res;
}

std::vector<material> *parser::get_materials()
{
	auto *materials = new std::vector<material>;

	if (!d["Scene"].IsObject() || !d["Scene"]["Materials"].IsObject())
	{
		my_printf("Warning: No 'Materials' object found in scene.\n");
		return materials;
	}

	const auto &materialsNode = d["Scene"]["Materials"];
	if (!materialsNode.HasMember("Material"))
	{
		return materials;
	}

	auto processMaterial = [&](const rapidjson::Value &mat_json)
	{
		auto parseVec3 = [](const char *str_val) -> vec3
		{
			std::istringstream iss(str_val);
			float r, g, b;
			iss >> r >> g >> b;
			return vec3(r, g, b);
		};

		material new_material;

		if (mat_json.HasMember("_type"))
		{
			std::string type_str = mat_json["_type"].GetString();
			if (type_str == "mirror")
			{
				new_material.mt = Mirror;
			}
			else if (type_str == "dielectric")
			{
				new_material.mt = Dielectric;
			}
			else if (type_str == "conductor")
			{
				new_material.mt = Conductor;
			}
			else
			{
				new_material.mt = Regular;
			}
		}

		if (mat_json.HasMember("AmbientReflectance"))
		{
			new_material.AmbientReflectance = parseVec3(mat_json["AmbientReflectance"].GetString());
		}
		if (mat_json.HasMember("DiffuseReflectance"))
		{
			new_material.DiffuseReflectance = parseVec3(mat_json["DiffuseReflectance"].GetString());
		}
		if (mat_json.HasMember("SpecularReflectance"))
		{
			new_material.SpecularReflectance = parseVec3(mat_json["SpecularReflectance"].GetString());
		}
		if (mat_json.HasMember("PhongExponent"))
		{
			new_material.PhongExponent = std::stof(mat_json["PhongExponent"].GetString());
		}

		if (mat_json.HasMember("MirrorReflectance"))
		{
			new_material.MirrorReflectance = parseVec3(mat_json["MirrorReflectance"].GetString());
		}
		if (mat_json.HasMember("AbsorptionCoefficient"))
		{
			new_material.AbsorptionCoefficient = parseVec3(mat_json["AbsorptionCoefficient"].GetString());
		}
		if (mat_json.HasMember("RefractionIndex"))
		{
			new_material.RefractionIndex = std::stof(mat_json["RefractionIndex"].GetString());
		}
		if (mat_json.HasMember("AbsorptionIndex"))
		{
			new_material.AbsorptionIndex = std::stof(mat_json["AbsorptionIndex"].GetString());
		}
		if (mat_json.HasMember("Roughness"))
		{
			new_material.roughness = std::stof(mat_json["Roughness"].GetString());
		}

		materials->emplace_back(new_material);
	};

	const auto &material_data = materialsNode["Material"];

	if (material_data.IsArray())
	{
		for (const auto &mat_item : material_data.GetArray())
		{
			processMaterial(mat_item);
		}
	}
	else if (material_data.IsObject())
	{
		processMaterial(material_data);
	}

	return materials;
}

std::vector<shape *> *parser::get_shapes(simd_vec3 &calculator, simd_mat4 &calculator_m, std::vector<vec3> *vertices, std::vector<material> *materials,
										 transformations *t, std::vector<texture *> *textures, std::vector<float> *uvs, std::vector<all_mesh_infos *> *m)
{
	auto *shapes = new std::vector<shape *>;

	if (!d["Scene"].IsObject() || !d["Scene"].HasMember("Objects"))
	{
		my_printf("Warning: No 'Objects' found in scene.\n");
		return shapes;
	}
	const auto &objects = d["Scene"]["Objects"];

	auto processMesh = [&](const rapidjson::Value &mesh)
	{
		if (!mesh.HasMember("Faces") || !(mesh["Faces"].HasMember("_data") || mesh["Faces"].HasMember("_plyFile")))
		{
			return;
		}

		material &mat = materials->at(std::stoi(mesh["Material"].GetString()) - 1);

		all_mesh_infos *ami = new all_mesh_infos();

		int mesh_id = std::stoi(mesh["_id"].GetString());

		m->push_back(ami);

		mesh_info m_info;
		m_info.mat = &mat;

		mat4 model;

		processTrans(mesh, calculator_m, t, model);

		m_info.model = model;
		calculator_m.inverse(model, m_info.inv_model);
		calculator_m.transpose(m_info.inv_model, m_info.normal);

		if (mesh.HasMember("Textures"))
		{
			std::istringstream iss(mesh["Textures"].GetString());
			int x;

			while (iss >> x)
			{
				for (auto &&i : *textures)
				{
					if (i->id == x)
					{
						m_info.textures.push_back(i);
					}
				}
			}
		}

		if (mesh.HasMember("MotionBlur"))
		{
			float px, py, pz;
			std::istringstream intStream(mesh["MotionBlur"].GetString());
			intStream >> px >> py >> pz;
			m_info.motionblur.load(px, py, pz);
		}

		ami->mesh_infos[mesh_id] = m_info;

		std::string shadingMode = "flat";
		if (mesh.HasMember("_shadingMode"))
			shadingMode = mesh["_shadingMode"].GetString();

		if (mesh["Faces"].HasMember("_plyFile"))
		{
			std::string plyName = mesh["Faces"]["_plyFile"].GetString();
			std::filesystem::path baseDir = std::filesystem::path(this->file_name).parent_path();
			std::filesystem::path fullPath = baseDir / plyName;
			load_ply(calculator, fullPath.string(), vertices, shapes, &mat, ami, shadingMode == "smooth");
		}
		else
		{
			std::istringstream iss(mesh["Faces"]["_data"].GetString());
			long long i0, i1, i2;
			std::vector<std::array<size_t, 3>> tris;
			while (iss >> i0 >> i1 >> i2)
			{
				tris.emplace_back(std::array<size_t, 3>{
					static_cast<size_t>(i0 - 1),
					static_cast<size_t>(i1 - 1),
					static_cast<size_t>(i2 - 1)});
			}

			if (shadingMode == "smooth")
			{
				std::vector<vec3> vertex_normals(vertices->size(), vec3(0.0f, 0.0f, 0.0f));

				for (auto &tri : tris)
				{
					vec3 e1, e2, n;
					calculator.subs(vertices->at(tri[1]), vertices->at(tri[0]), e1);
					calculator.subs(vertices->at(tri[2]), vertices->at(tri[0]), e2);
					calculator.cross(e1, e2, n);

					calculator.add(vertex_normals[tri[0]], n, vertex_normals[tri[0]]);
					calculator.add(vertex_normals[tri[1]], n, vertex_normals[tri[1]]);
					calculator.add(vertex_normals[tri[2]], n, vertex_normals[tri[2]]);
				}

				for (auto &vn : vertex_normals)
					calculator.normalize(vn, vn);

				for (auto &tri : tris)
				{
					shapes->push_back(new triangle(
						calculator,
						&vertices->at(tri[0]),
						&vertices->at(tri[1]),
						&vertices->at(tri[2]),
						uvs->size() == 0 ? vec3() : vec3(uvs->at(tri[0] * 2), uvs->at(tri[1] * 2), uvs->at(tri[2] * 2)),
						uvs->size() == 0 ? vec3() : vec3(uvs->at(tri[0] * 2 + 1), uvs->at(tri[1] * 2 + 1), uvs->at(tri[2] * 2 + 1)),
						vertex_normals[tri[0]],
						vertex_normals[tri[1]],
						vertex_normals[tri[2]],
						&mat, ami));
				}
			}
			else
			{
				for (auto &tri : tris)
				{
					shapes->push_back(new triangle(
						calculator,
						&vertices->at(tri[0]),
						&vertices->at(tri[1]),
						&vertices->at(tri[2]),
						uvs->size() == 0 ? vec3() : vec3(uvs->at(tri[0] * 2), uvs->at(tri[1] * 2), uvs->at(tri[2] * 2)),
						uvs->size() == 0 ? vec3() : vec3(uvs->at(tri[0] * 2 + 1), uvs->at(tri[1] * 2 + 1), uvs->at(tri[2] * 2 + 1)),
						&mat, ami));
				}
			}
		}
	};

	auto processMeshInstance = [&](const rapidjson::Value &MeshInstance)
	{
		int id = std::stoi(MeshInstance["_id"].GetString());
		int base_id = std::stoi(MeshInstance["_baseMeshId"].GetString());
		bool reset = false;
		reset = MeshInstance.HasMember("_resetTransform") && (strcmp(MeshInstance["_resetTransform"].GetString(), "true") == 0);
		material *mat = 0;
		if (MeshInstance.HasMember("Material"))
		{
			mat = &materials->at(std::stoi(MeshInstance["Material"].GetString()) - 1);
		}
		mat4 model;
		processTrans(MeshInstance, calculator_m, t, model);
		all_mesh_infos *target = 0;
		for (auto &&i : *m)
		{
			if (i->mesh_infos.contains(base_id))
			{
				target = i;
				break;
			}
		}
		if (target == 0)
		{
			my_printf("MeshInstance can't find base mesh.\n");
			exit(-1);
		}
		if (!reset)
		{
			calculator_m.mult(model, target->mesh_infos[base_id].model, model);
		}
		mesh_info m_info;
		if (mat == 0)
		{
			mat = target->mesh_infos[base_id].mat;
		}
		m_info.mat = mat;
		m_info.model = model;
		calculator_m.inverse(model, m_info.inv_model);
		calculator_m.transpose(m_info.inv_model, m_info.normal);

		if (MeshInstance.HasMember("Textures"))
		{
			std::istringstream iss(MeshInstance["Textures"].GetString());
			int x;

			while (iss >> x)
			{
				for (auto &&i : *textures)
				{
					if (i->id == x)
					{
						m_info.textures.push_back(i);
					}
				}
			}
		}

		if (MeshInstance.HasMember("MotionBlur"))
		{
			float px, py, pz;
			std::istringstream intStream(MeshInstance["MotionBlur"].GetString());
			intStream >> px >> py >> pz;
			m_info.motionblur.load(px, py, pz);
		}

		target->mesh_infos[id] = m_info;
		my_printf("Mesh instance id: %d added for base id: %d\n", id, base_id);
	};

	auto processTriangle = [&](const rapidjson::Value &tri)
	{
		if (!tri.HasMember("Indices"))
			return;

		std::istringstream iss(tri["Indices"].GetString());
		long long i0, i1, i2;

		auto &mat = materials->at(std::stoi(tri["Material"].GetString()) - 1);

		all_mesh_infos *ami = new all_mesh_infos();

		int mesh_id = 123456;

		m->push_back(ami);

		mesh_info m_info;
		m_info.mat = &mat;

		mat4 model;

		processTrans(tri, calculator_m, t, model);

		m_info.model = model;
		calculator_m.inverse(model, m_info.inv_model);
		calculator_m.transpose(m_info.inv_model, m_info.normal);

		if (tri.HasMember("Textures"))
		{
			std::istringstream iss(tri["Textures"].GetString());
			int x;

			while (iss >> x)
			{
				for (auto &&i : *textures)
				{
					if (i->id == x)
					{
						m_info.textures.push_back(i);
					}
				}
			}
		}

		ami->mesh_infos[mesh_id] = m_info;

		if (iss >> i0 >> i1 >> i2)
		{
			shapes->push_back(new triangle(
				calculator,
				&vertices->at(static_cast<size_t>(i0 - 1)),
				&vertices->at(static_cast<size_t>(i1 - 1)),
				&vertices->at(static_cast<size_t>(i2 - 1)),
				uvs->size() == 0 ? vec3() : vec3(uvs->at((i0 - 1) * 2), uvs->at((i1 - 1) * 2), uvs->at((i2 - 1) * 2)),
				uvs->size() == 0 ? vec3() : vec3(uvs->at((i0 - 1) * 2 + 1), uvs->at((i1 - 1) * 2 + 1), uvs->at((i2 - 1) * 2 + 1)),
				&mat,
				ami));
		}
	};

	auto processSphere = [&](const rapidjson::Value &sph)
	{
		if (!sph.HasMember("Center") || !sph.HasMember("Radius"))
			return;

		int center_idx = std::stoi(sph["Center"].GetString());
		float radius = std::stof(sph["Radius"].GetString());

		mat4 model;

		processTrans(sph, calculator_m, t, model);

		mat4 inv;

		calculator_m.inverse(model, inv);

		mat4 normal;

		calculator_m.transpose(inv, normal);

		std::vector<texture *> texturesx;
		if (sph.HasMember("Textures"))
		{
			std::istringstream iss(sph["Textures"].GetString());
			int x;

			while (iss >> x)
			{
				for (auto &&i : *textures)
				{
					if (i->id == x)
					{
						texturesx.push_back(i);
					}
				}
			}
		}

		shapes->push_back(new sphere(
			calculator, calculator_m,
			&vertices->at(center_idx - 1),
			radius,
			&materials->at(std::stoi(sph["Material"].GetString()) - 1), texturesx, model, inv, normal));
	};

	auto processPlane = [&](const rapidjson::Value &pl)
	{
		if (!pl.HasMember("Point") || !pl.HasMember("Normal"))
			return;

		int point_idx = std::stoi(pl["Point"].GetString());

		std::istringstream iss(pl["Normal"].GetString());
		float nx, ny, nz;
		iss >> nx >> ny >> nz;

		vec3 normal_vec(nx, ny, nz);

		mat4 model;

		processTrans(pl, calculator_m, t, model);

		vec3 transformedPoint;
		calculator_m.mult_vec(model, vertices->at(point_idx - 1), transformedPoint, false);
		mat4 norm_m;
		calculator_m.inverse(model, norm_m);
		calculator_m.transpose(norm_m, norm_m);
		calculator_m.mult_vec(norm_m, normal_vec, normal_vec, true);
		calculator.normalize(normal_vec, normal_vec);

		std::vector<texture *> texturesx;
		if (pl.HasMember("Textures"))
		{
			std::istringstream iss(pl["Textures"].GetString());
			int x;

			while (iss >> x)
			{
				for (auto &&i : *textures)
				{
					if (i->id == x)
					{
						texturesx.push_back(i);
					}
				}
			}
		}

		shapes->push_back(new plane(
			&transformedPoint,
			&normal_vec,
			&materials->at(std::stoi(pl["Material"].GetString()) - 1), texturesx));
	};

	size_t total_extra_vertices = 0;
	if (objects.HasMember("Mesh"))
	{
		const auto &node = objects["Mesh"];
		auto checkNode = [&](const rapidjson::Value &item)
		{
			if (!item.HasMember("Faces") || !item["Faces"].HasMember("_plyFile"))
				return;
			std::string plyName = item["Faces"]["_plyFile"].GetString();
			std::filesystem::path baseDir = std::filesystem::path(this->file_name).parent_path();
			std::filesystem::path fullPath = baseDir / plyName;
			std::ifstream pf(fullPath, std::ios::binary);
			if (!pf.is_open())
			{
				my_printf("Warning: could not open PLY header for reserve: %s\n", fullPath.string().c_str());
				return;
			}
			std::string line;
			while (std::getline(pf, line))
			{
				size_t a = line.find_first_not_of(" \t\r\n");
				if (a == std::string::npos)
					continue;
				if (line.rfind("element vertex", 0) == 0)
				{
					std::istringstream iss(line);
					std::string tmp;
					size_t cnt = 0;
					iss >> tmp >> tmp >> cnt;
					total_extra_vertices += cnt;
					break;
				}
				if (line == "end_header")
					break;
			}
			pf.close();
		};

		if (node.IsArray())
		{
			for (const auto &item : node.GetArray())
				checkNode(item);
		}
		else if (node.IsObject())
		{
			checkNode(node);
		}
	}

	if (total_extra_vertices > 0)
	{
		vertices->reserve(vertices->size() + total_extra_vertices);
		my_printf("DEBUG: reserved vertices capacity: %zu (+%zu for meshes)\n", vertices->capacity(), total_extra_vertices);
	}

	if (objects.HasMember("Mesh"))
	{
		const auto &node = objects["Mesh"];
		if (node.IsArray())
		{
			for (const auto &item : node.GetArray())
			{
				processMesh(item);
			}
		}
		else if (node.IsObject())
		{
			processMesh(node);
		}
	}

	if (objects.HasMember("MeshInstance"))
	{
		const auto &node = objects["MeshInstance"];
		if (node.IsArray())
		{
			for (const auto &item : node.GetArray())
			{
				processMeshInstance(item);
			}
		}
		else if (node.IsObject())
		{
			processMeshInstance(node);
		}
	}

	if (objects.HasMember("Triangle"))
	{
		const auto &node = objects["Triangle"];
		if (node.IsArray())
		{
			for (const auto &item : node.GetArray())
			{
				processTriangle(item);
			}
		}
		else if (node.IsObject())
		{
			processTriangle(node);
		}
	}

	if (objects.HasMember("Sphere"))
	{
		const auto &node = objects["Sphere"];
		if (node.IsArray())
		{
			for (const auto &item : node.GetArray())
			{
				processSphere(item);
			}
		}
		else if (node.IsObject())
		{
			processSphere(node);
		}
	}

	if (objects.HasMember("Plane"))
	{
		const auto &node = objects["Plane"];
		if (node.IsArray())
		{
			for (const auto &item : node.GetArray())
			{
				processPlane(item);
			}
		}
		else if (node.IsObject())
		{
			processPlane(node);
		}
	}

	return shapes;
}

float parser::get_intersectionepsilon()
{
	if (d["Scene"].HasMember("IntersectionTestEpsilon"))
		return std::stof(d["Scene"]["IntersectionTestEpsilon"].GetString());
	return 1e-6f;
}

float parser::get_shadowrayepsilon()
{
	if (d["Scene"].HasMember("ShadowRayEpsilon"))
		return std::stof(d["Scene"]["ShadowRayEpsilon"].GetString());
	return 1e-3f;
}

float parser::get_maxrecursiondepth()
{
	if (d["Scene"].HasMember("MaxRecursionDepth"))
		return std::stof(d["Scene"]["MaxRecursionDepth"].GetString());
	return 6;
}

vec3 parser::get_backgroundcolor()
{
	vec3 color;
	if (d["Scene"].HasMember("BackgroundColor"))
	{
		std::string tmp = d["Scene"]["BackgroundColor"].GetString();
		std::istringstream iss(tmp);
		float r, g, b;
		iss >> r >> g >> b;
		color.load(r, g, b);
	}
	else
		color.load(0, 0, 0);
	return color;
}

vec3 parser::get_ambientlight()
{
	vec3 amb;
	if (d["Scene"].HasMember("Lights") && d["Scene"]["Lights"].HasMember("AmbientLight"))
	{
		std::string tmp = d["Scene"]["Lights"]["AmbientLight"].GetString();
		std::istringstream iss(tmp);
		float r, g, b;
		iss >> r >> g >> b;
		amb.load(r, g, b);
	}
	else
		amb.load(0, 0, 0);
	return amb;
}

std::vector<Light *> *parser::get_pointlights(simd_vec3 &calculator, simd_mat4 &calculator_m, transformations *t)
{
	std::vector<Light *> *lights = new std::vector<Light *>();

	if (!d["Scene"].HasMember("Lights"))
		return lights;

	if (d["Scene"]["Lights"].HasMember("PointLight"))
	{
		const auto &plNode = d["Scene"]["Lights"]["PointLight"];

		auto processPL = [&](const rapidjson::Value &plNode)
		{
			vec3 position, intensity;

			std::string tmp = plNode["Position"].GetString();
			std::istringstream posStream(tmp);
			float px, py, pz;
			posStream >> px >> py >> pz;
			position.load(px, py, pz);
			mat4 model;
			processTrans(plNode, calculator_m, t, model);
			calculator_m.mult_vec(model, position, position, false);

			tmp = plNode["Intensity"].GetString();
			std::istringstream intStream(tmp);
			intStream >> px >> py >> pz;
			intensity.load(px, py, pz);

			PointLight *light = new PointLight(position, intensity);

			lights->push_back(light);
		};

		if (plNode.IsObject())
		{
			processPL(plNode);
		}
		else if (plNode.IsArray())
		{
			for (auto &v : plNode.GetArray())
			{
				processPL(v);
			}
		}
	}

	if (d["Scene"]["Lights"].HasMember("AreaLight"))
	{
		const auto &alNode = d["Scene"]["Lights"]["AreaLight"];

		auto processAL = [&](const rapidjson::Value &plNode)
		{
			vec3 position, normal, radiance;

			std::string tmp = plNode["Position"].GetString();
			std::istringstream posStream(tmp);
			float px, py, pz;
			posStream >> px >> py >> pz;
			position.load(px, py, pz);
			mat4 model;
			processTrans(plNode, calculator_m, t, model);
			calculator_m.mult_vec(model, position, position, false);

			tmp = plNode["Normal"].GetString();
			std::istringstream nStream(tmp);
			nStream >> px >> py >> pz;
			normal.load(px, py, pz);

			tmp = plNode["Radiance"].GetString();
			std::istringstream rStream(tmp);
			rStream >> px >> py >> pz;
			radiance.load(px, py, pz);

			px = std::stof(plNode["Size"].GetString());

			AreaLight *light = new AreaLight(calculator, position, normal, px, radiance);

			lights->push_back(light);
		};

		if (alNode.IsObject())
		{
			processAL(alNode);
		}
		else if (alNode.IsArray())
		{
			for (auto &v : alNode.GetArray())
			{
				processAL(v);
			}
		}
	}

	return lights;
}

constexpr float to_radians(float degrees)
{
	return degrees * (3.1415926535897932384626433832 / 180.0f);
}

transformations *parser::get_transformations(simd_mat4 &calculator)
{
	transformations *res = new transformations;

	if (!d["Scene"].HasMember("Transformations"))
		return res;

	const rapidjson::Value &transforms = d["Scene"]["Transformations"];

	if (!transforms.IsObject())
		return res;

	std::string tmp;

	auto parse_transformation_list = [&](const char *name,
										 std::vector<mat4> &matrix_list,
										 auto matrix_generator) // Lambda for matrix generation
	{
		if (transforms.HasMember(name))
		{
			const rapidjson::Value &node = transforms[name];

			auto parse_single_transform = [&](const rapidjson::Value &trans_node)
			{
				if (!trans_node.IsObject() || !trans_node.HasMember("_id") || !trans_node.HasMember("_data"))
					return;

				tmp = trans_node["_data"].GetString();
				mat4 m;
				matrix_generator(tmp, m);
				matrix_list.push_back(m);
			};

			if (node.IsArray())
				for (const auto &item : node.GetArray())
					parse_single_transform(item);
			else if (node.IsObject())
				parse_single_transform(node);
		}
	};

	parse_transformation_list("Translation", res->translations,
							  [&](const std::string &data_str, mat4 &m)
							  {
								  float tx, ty, tz;
								  sscanf(data_str.c_str(), "%f %f %f", &tx, &ty, &tz);
								  vec3 t(tx, ty, tz);
								  calculator.translate(t, m);
							  });
	parse_transformation_list("Rotation", res->rotations,
							  [&](const std::string &data_str, mat4 &m)
							  {
								  float angle_deg, rx, ry, rz;
								  sscanf(data_str.c_str(), "%f %f %f %f", &angle_deg, &rx, &ry, &rz);
								  vec3 r(rx, ry, rz);
								  float angle_rad = to_radians(angle_deg);
								  calculator.rotate(r, angle_rad, m);
							  });
	parse_transformation_list("Scaling", res->scales,
							  [&](const std::string &data_str, mat4 &m)
							  {
								  float sx, sy, sz;
								  sscanf(data_str.c_str(), "%f %f %f", &sx, &sy, &sz);
								  vec3 s(sx, sy, sz);
								  calculator.scale(s, m);
							  });

	return res;
}