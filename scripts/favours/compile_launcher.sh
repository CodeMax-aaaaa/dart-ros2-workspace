#!/bin/bash

colcon build --merge-install \
    --cmake-args \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -G Ninja \
    --event-handlers console_direct+ \
    --packages-select \
    cv_bridge dart_msgs dart_detector dart_launcher
