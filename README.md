# Raytracer

Simple path tracer / raytracer.

Blog: [https://blog.sukruciris.com/raytracer.html](https://blog.sukruciris.com/raytracer.html)

---

# How to Build

```bash
make
```

---

# Video Functionality

This project can automatically create videos from numbered PNG sequences (e.g. `animation_0000.png`, `animation_0001.png`, ...). It uses a bundled FFmpeg executable which **must** be placed into the repository next to the raytracer binary in the following folders:

* Windows: `win_ffmpeg/ffmpeg.exe`
* Linux/macOS: `linux_ffmpeg/ffmpeg`

The program looks for the FFmpeg executable using a relative path from the current working directory. After successfully creating a video from a PNG sequence, the source PNG files are moved to `outputs/used/`.

---

# How to Run

```
./raytracer <input_path> [threading_type] [thread_count]
```

* `<input_path>` — path to a `.json` scene file **or** a directory. If a directory is provided, all `.json` files inside it (recursively) are processed.
* `[threading_type]` — **optional** integer that determines how rendering work is divided among threads.
* `[thread_count]` — **optional** integer setting the number of worker threads.

**Notes on defaults and behavior**

* If only `<input_path>` is provided, the program uses all available hardware threads and the default threading strategy (`row_dynamic`).
* If the `threading_type` is provided but `thread_count` is omitted, the program still uses automatic thread count (hardware concurrency).
* If both `threading_type` and `thread_count` are provided, the program uses the supplied `thread_count`.
* Valid `thread_count` values are positive integers. The program enforces `1 <= thread_count <= 4096`. Invalid values cause the program to exit with an error.

---

# Threading Types

Pass the integer below as `threading_type` to choose the scheduling strategy:

| Value | Name             | Description                                                                                  |
| ----- | ---------------- | -------------------------------------------------------------------------------------------- |
| `0`   | `row_dynamic`    | Threads take the next row dynamically (default).                                             |
| `1`   | `row_static`     | Static division of the frame into horizontal stripes — each thread gets a fixed set of rows. |
| `2`   | `square_dynamic` | Threads take the next 32×32 tile dynamically (tile-based scheduling).                        |
| `3`   | `pixel_dynamic`  | Threads take the next single pixel dynamically.                                              |

---

# Examples

* Use automatic thread count and default threading:

  ```bash
  ./raytracer scene.json
  ```

* Choose threading strategy, use automatic thread count:

  ```bash
  ./raytracer scene.json 1
  # uses row_static with hardware_concurrency() threads
  ```

* Fully custom (directory of scenes, square dynamic scheduling, 8 threads):

  ```bash
  ./raytracer ./scenes/ 2 8
  ```

---

# Output

* Rendered images and generated videos will appear in the `outputs/` folder.
* HDR outputs (e.g. `.exr`) are written when a camera is configured for HDR. Tone mapping is applied according to camera settings in the JSON; multiple tone-mapped outputs may be produced with different extensions as configured.
* After video creation, the source PNG files (that made the video) are moved into `outputs/used/`.

---

# Error & Validation

* If the provided `input_path` is neither a `.json` file nor a directory, the program exits with an error.
* If an invalid `threading_type` value is given (outside `0..3`), the program prints `Invalid threading type` and exits.
* If `thread_count` is non-positive or greater than 4096, the program prints `Invalid thread count` or exits with an error code.

---

# Tips

* Use `row_dynamic` (default) or `square_dynamic` for good load balancing on complex scenes.
* For very small images, `pixel_dynamic` can work well; for very large images, `square_dynamic` can have less contention.
* Ensure FFmpeg executable path is correct relative to your current working directory when using the video feature.

---

# Author

Şükrü Çiriş

---