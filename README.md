# 3rd-year-project
An Object Detection and Tracking System for Drone Navigation using a RYZE Tello drone, and OpenCV in C++.

## Dependencies
This project was developed and tested with the versions listed below.
| Name         |        Version         |                    Notes                     |
|--------------|:----------------------:|:--------------------------------------------:|
| C++          |           17           |                                              |
| C++ compiler |        unknown         |                                              |
| CMake        |          3.25          |                                              |
| spdlog       |        1.5.0-1         |                                              |
| OpenCV       |          4.6.0         | With `opencv_contrib` installed              |
| CTello       |          Latest        | [Link](https://github.com/carlospzlz/ctello) |

## Compilation
### Full System
```
mkdir build && cd build
cmake ..
cmake --build .
./tracking-drone
```

### OpenCV Code
In order to compile the files containing the OpenCV library, such as those in the `test-code` folder, use the following options:

Compile using the script provided:
```
./opencv-compile IN-FILE.cpp OUT-file
```
OR
```
bash opencv-compile IN-FILE.cpp OUT-file
```
OR

Compile yourself using the command below or however you would compile an openCV cpp file.
```
g++ m.cpp -o app `pkg-config --cflags --libs opencv4`
```
