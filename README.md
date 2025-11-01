# How to Build

```
make
```

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

The program outputs results to the terminal and also saves them to output.txt for later inspection.

# Şükrü Çiriş