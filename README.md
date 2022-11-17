# 3rd-year-project
Drone object tracking using a RYZE Tello drone, and OpenCV in C++.

## Requirements
- CTello - Available at: https://github.com/carlospzlz/ctello
- CMake (Developed using CMake 3.25)
- OpenCV (Developed using OpenCV 4.6 with opencv_contrib installed) 

## Compilation
### Full System
```
mkdir build && cd build
cmake ..
cmake --build .
./drone-test
```

### OpenCV Code
```
g++ m.cpp -o app `pkg-config --cflags --libs opencv`
```