#!/bin/bash
if [[ $# -ne 2 ]]; then
    echo "usage: $0 IN-FILE.cpp OUT-FILE" >&2
    exit 2
fi
g++ $1 -o $2 `pkg-config --cflags --libs opencv4`