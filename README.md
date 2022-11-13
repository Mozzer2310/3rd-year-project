# 3rd-year-project
Drone object tracking

## Compilation
```
g++ -std=c++17 -I/usr/local/include -L/usr/local/lib program.cpp -lctello -o program `pkg-config --cflags --libs opencv4`
```