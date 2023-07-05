# Jetbot-line-following-program
A python program built for the Jetson Nano which uses the built-in camera to detect a path and follow it.

# Process for running the code
1. Open a CLI
2. Run the following commands:

    `cd jetbot/jetbot`

    `./test-launch "nvarguscamerasrc ! video/x-raw(memory:NVMM),width=600,height=480,framerate=15/1,format=NV12 ! nvvidconv ! nvv4l2h265enc ! h265parse ! rtph265pay name=pay0 pt=96"`

This will start the RTSP stream.

3. Open a separate CLI while the first is still running
4. Run the following commands:

    `cd jetbot/jetbot`

    `python3 contourtest.py`

This will run the code which starts the line following protocol.

To Run the incomplete Object Detection Program, run:
    
    python3 yoloCustomObjectDetection.py

To run the Distance Measuring Program, run:

    cd examples
    python3 VL53.py
