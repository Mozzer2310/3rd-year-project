# 3rd-year-project
An Object Tracking System for Drone Navigation using a RYZE Tello drone, and OpenCV in C++.

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

## File Explanation
### `tracking-drone.cpp`
This is the file containing the code for my implementation of the project. Compiling and running this file, gives a system which automatically controls a RYZE Tello drone connected over WiFi to track and follow a user defined object/region of interest.

### `object-tracking.cpp`
This file is very similar to the full application, but with all of the drone controls removed. This allows the evaluation and testing of the object tracking system and drone command generation without having a drone connected. It is used as a testing and evaluation file, as connected and controlling a drone is time consuming.

The drone commands are outputted as strings in the console, and the overlays to show movement are the same as the `tracking-drone.cpp` file.

## Compilation
### Full System: `tracking-drone.cpp`
```
mkdir build && cd build
cmake ..
cmake --build .
./tracking-drone
```

### OpenCV Code: Archive files and `object-tracking.cpp`
In order to compile the files containing the OpenCV library, use the following options:

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
