#!/bin/bash

if [ "$1" != "left" ] && [ "$1" != "right" ]; then
    echo "Usage: ./img_grid.sh [left|right]"
    exit 1
fi

python3 img_grid.py $1