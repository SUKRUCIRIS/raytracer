# Blog

https://blog.sukruciris.com/raytracer.html

# How to Build

```
make
```

# Video Functionality

This project uses the FFmpeg executable to create videos from numbered image files. Since the FFmpeg binaries were too large to include in the repository, you’ll need to add them manually. On Windows, create a `win_ffmpeg` folder in the project directory and place the FFmpeg executable inside. On Linux, do the same with a `linux_ffmpeg` folder. The required folders must be placed in the same directory as the raytracer executable.

# How to Run

You can give json file name as first argument and thread count as second argument:
```
./raytracer input.json thread_count
```

Or you can leave thread count empty and program will launch optimal thread count for your system as default:
```
./raytracer input.json
```

To process all JSON files in a folder, simply provide the folder path:
```
./raytracer ./inputs/
```

The program outputs results to the terminal and also saves them to output.txt for later inspection. Resulting images and videos will be in the outputs folder.

# Şükrü Çiriş