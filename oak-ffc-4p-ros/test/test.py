#!/usr/bin/env python3

import cv2
import depthai as dai

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
monoA = pipeline.create(dai.node.MonoCamera)
monoD = pipeline.create(dai.node.MonoCamera)
RgbB = pipeline.create(dai.node.ColorCamera)
RgbC = pipeline.create(dai.node.ColorCamera)
xoutA = pipeline.create(dai.node.XLinkOut)
xoutD = pipeline.create(dai.node.XLinkOut)
xoutB = pipeline.create(dai.node.XLinkOut)
xoutC = pipeline.create(dai.node.XLinkOut)

xoutA.setStreamName('rgb')
xoutD.setStreamName('camd')
xoutB.setStreamName('camb')
xoutC.setStreamName('camc')

# Properties
monoA.setBoardSocket(dai.CameraBoardSocket.CAM_A)
monoA.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
monoD.setBoardSocket(dai.CameraBoardSocket.CAM_D)
monoD.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
RgbB.setBoardSocket(dai.CameraBoardSocket.CAM_B)
RgbB.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
RgbC.setBoardSocket(dai.CameraBoardSocket.CAM_C)
RgbC.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)


# Linking
monoA.out.link(xoutA.input)
monoD.out.link(xoutD.input)
RgbB.video.link(xoutB.input)
RgbC.video.link(xoutC.input)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    # Output queues will be used to get the grayscale frames from the outputs defined above
    qCamA = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    qCamD = device.getOutputQueue(name="camd", maxSize=4, blocking=False)
    qCamB = device.getOutputQueue(name="camb", maxSize=4, blocking=False)
    qCamC = device.getOutputQueue(name="camc", maxSize=4, blocking=False)

    while True:
        # Instead of get (blocking), we use tryGet (non-blocking) which will return the available data or None otherwise
        inCamA = qCamA.tryGet()
        inCamD = qCamD.tryGet()
        inCamB = qCamB.tryGet()
        inCamC = qCamC.tryGet()

        if inCamA is not None:
            cv2.imshow("rgb", inCamA.getCvFrame())

        if inCamD is not None:
            cv2.imshow("camd", inCamD.getCvFrame())
            
        if inCamB is not None:
            cv2.imshow("camb", inCamB.getCvFrame())

        if inCamC is not None:
            cv2.imshow("camc", inCamC.getCvFrame())

        if cv2.waitKey(1) == ord('q'):
            break
