import argparse
import os
import cv2
import re
import math
import numpy as np
import warnings

import OpticalNavigation.core.img_preprocess as ip
import OpticalNavigation.core.find as find
import OpticalNavigation.core.camera as camera

#import img_preprocess as ip
#import find


# import camera

# Adapted from https://www.geeksforgeeks.org/extract-images-from-video-in-python/
def mjpegToJpeg(inputFile):
    """Convertes and mjpeg video into a list of jpeg images"""
    src = cv2.VideoCapture(inputFile)
    currentFrame = 0
    base = os.path.splitext(inputFile)[0]
    frames = []
    while (True):
    # Write next frame to a new jpeg image
        ret, frame = src.read()
        if ret:
            name = base + f"_f{currentFrame}.jpeg"
            cv2.imwrite(name, frame)
            frames.append(name)
            currentFrame += 1
        else:
            break
    # src.release()
    # cv2.destroyAllWindows()
    return frames


def observe():
    """The entire obeserve stage of opnav.
    Performs video capture, image preprocessing, object detection
    Intermediate return: original timestamp, 3-dimensional coordinates and angular diameter of sun, earth, moon on each of three cameras
    Final return is a vector of the sun, earth and moon each at the time of recording in a unified body frame

    File naming convention:
    Video: cam{number}_exp{High/Low}.mjpeg
    Image: cam{number}_exp{High/Low}_f{number}.jpeg

    TODO: Need to rotate vectors and bring back to first timestamp

    Output:
    timestamp, coordinate, radius, camNum, filename
"""

    # Dict of filenames and associated final timestamps


    recordings = {}

# To use on flight hardware - records videos from picameras
    mux = camera.CameraMux()
    for i in [1, 2, 3]:  # Make sure that these numbers correspond ot the ports on the hardware
    # Mux and camera initialization
        mux.selectCamera(i)
        cam = camera.Camera()
        ##framerate_tmp = 15

        # Records exposure sequence 1 - shutterSpeed value
        file1, timestamp1 = cam.rawObservation(f"cam{i}_expHigh.mjpeg", shutterSpeed=50000)  # add shutterspeed value

        # Records exposure sequence 2
        file2, timestamp2 = cam.rawObservation(f"cam{i}_expLow.mjpeg", shutterSpeed=20000)

        recordings[file1] = timestamp1
        recordings[file2] = timestamp2

    print("Data retrieved from cameras:")
    print(recordings)

    # To use when not on flight hardware - inserts replacement images for analysis
    # Final timestamp based on 5 frame 15fps video (times are in microseconds)
    # recordings["cam1_expHigh_rep.mjpeg"] = 333330  # Sun video
    # recordings["cam1_expLow_rep.mjpeg"]  = 333330  # Sun video
    # recordings["cam2_expHigh_rep.mjpeg"] = 333330  # Earth video
    # recordings["cam2_expLow_rep.mjpeg"]  = 333330  # Earth video
    # del recordings["cam1_expHigh.mjpeg"]
    # del recordings["cam1_expLow.mjpeg"]
    # del recordings["cam2_expHigh.mjpeg"]
    # del recordings["cam2_expLow.mjpeg"]
    # del recordings["cam3_expHigh.mjpeg"]
    # del recordings["cam3_expLow.mjpeg"]

    # Three arrays for return data
    earthData = []
    moonData = []
    sunData = []

    # Iterate through each video
    for filename, timestamp in recordings.items():
        frames = mjpegToJpeg(filename)

        # imgData = [sun, earth, moon, image, framename]
        # where celestial body = [x, y, z, radius]
        imgData = []

        # Extract find data for each frame, put into imgData[]
        for i in frames:
            ip.preprocess(i, i)
            img = cv2.imread(i)
            s, e, m, img = find.find(img)  # Make sure find is updated with xyz vectors
            data = [s, e, m, img, i]
            imgData.append(data)

        # centerX = 320  # Need to maintain with camera parameters
        # centerY = 240

        # Extract best sun match and place in sunData
        exposureHigh = "High" in filename
        if exposureHigh:
            sunDist = 1000000  # Some large number
            sunImage = None
            for i in imgData:
                if i[0] is not None:  # Sun data
                    # sunXdiff = abs(i[0][0] - centerX)
                    # sunYdiff = abs(i[0][1] - centerY)
                    #                           x                 y
                    distToCenter = math.sqrt((i[0][0] ** 2) + (i[0][1] ** 2))
                    if distToCenter < sunDist:
                        sunDist = distToCenter
                        sunImage = i;

                    # sunImage is now the frame with sun closest to the z-axis
            if sunImage is not None:
                # Extract sun frame number from sunImage
                sunFrameNum = int(re.search("[f](\d+)", sunImage[4]).group(1))
                sunCamNum = int(re.search("[cam](\d+)", sunImage[4]).group(1))
                # Calculate timestamp from framenumber and last timestamp of recording
                sunFrameTimestamp = timestamp - (len(frames) - 1 - sunFrameNum) * (1 / "fps" * 10 ** 6)

                sunDataGroup = []
                sunDataGroup.append(sunFrameTimestamp)  # Timestamp
                sunDataGroup.append(sunImage[0][0:3])  # Coordinate
                sunDataGroup.append(sunImage[0][3])  # Radius
                sunDataGroup.append(sunCamNum)  # Camera Number
                sunDataGroup.append(sunImage[4])  # Filename for debugging
                sunData.append(sunDataGroup)

            # Now we extract best earth and moon matches and place in earthData, moonData
        exposureLow = "Low" in filename
        if exposureLow:  # is low exposure (earth/moon)
            earthDist = 1000000  # Some large number
            moonDist = 1000000  # Some large number
            earthImage = None
            moonImage = None
            for i in imgData:
                # Earth
                if i[1] is not None:
                    # earthXdiff = abs(i[1][0] - centerX)
                    # earthYdiff = abs(i[1][1] - centerY)
                    #
                    earthDistToCenter = math.sqrt((i[1][0] ** 2) + (i[1][1] ** 2))
                    if earthDistToCenter < earthDist:
                        earthDist = earthDistToCenter
                        earthImage = i;
                    # Moon
                if i[2] is not None:
                    # moonXdiff = abs(i[2][0] - centerX)
                    # moonYdiff = abs(i[2][1] - centerY)
                    #
                    moonDistToCenter = math.sqrt((i[2][0] ** 2) + (i[2][1] ** 2))
                    if moonDistToCenter < moonDist:
                        moonDist = moonDistToCenter
                        moonImage = i;

                    # earthImage is now the frame with earth closest to the z-axis
            if earthImage is not None:
                # Extract earth frame number from earthImage
                earthFrameNum = int(re.search("[f](\d+)", earthImage[4]).group(1))
                earthCamNum = int(re.search("[cam](\d+)", earthImage[4]).group(1))
                # Calculate timestamp from framenumber and last timestamp of recording
                earthFrameTimestamp = timestamp - (len(frames) - 1 - earthFrameNum) * (1 / "fps" * 10 ** 6)

                earthDataGroup = []
                earthDataGroup.append(earthFrameTimestamp)  # Timestamp
                earthDataGroup.append(earthImage[1][0:3])  # Coordinate
                earthDataGroup.append(earthImage[1][3])  # Radius
                earthDataGroup.append(earthCamNum)  # Camera Number
                earthDataGroup.append(earthImage[4])  # Filename for debugging
                earthData.append(earthDataGroup)

            # moonImage is now the frame with moon closest to the z-axis
            if moonImage is not None:
                # Extract moon frame number from moonimage
                moonFrameNum = int(re.search("[f](\d+)", moonImage[4]).group(1))
                moonCamNum = int(re.search("[cam](\d+)", moonImage[4]).group(1))
                # Calculate timestamp from framenumber and last timestamp of recording
                moonFrameTimestamp = timestamp - (len(frames) - 1 - moonFrameNum) * (
                            1 / "fps" * 10 ** 6)  # FPS is denominator

                moonDataGroup = []
                moonDataGroup.append(moonFrameTimestamp)  # Timestamp
                moonDataGroup.append(moonImage[0][0:3])  # Coordinate
                moonDataGroup.append(moonImage[0][3])  # Radius
                moonDataGroup.append(moonCamNum)  # Camera Number
                moonDataGroup.append(moonImage[4])  # Filename for debugging
                moonData.append(moonDataGroup)

        cam1Rotation = np.array([0, 1, 0, 0.5, 0, -1 * math.sqrt(3) / 2, -1 * math.sqrt(3) / 2, 0, -1 / 2]).reshape(3, 3)
        cam2Rotation = np.array([0, 1, 0, 0.5, 0, math.sqrt(3) / 2, math.sqrt(3) / 2, 0, -1 / 2]).reshape(3, 3)
        cam3Rotation = np.array([math.sqrt(2) / 2, math.sqrt(2) / 2, 0, math.sqrt(2) / 2, -1 * math.sqrt(2) / 2, 0, 0, 0, 1]).reshape(3,3)

        for data in sunData:
            coordArray = np.array([data[1][0], data[1][1], data[1][2]]).reshape(3, 1)
            if data[3] == 1:
                coordArray = cam1Rotation.dot(coordArray)
            elif data[3] == 2:
                coordArray = cam2Rotation.dot(coordArray)
            elif data[3] == 3:
                coordArray = cam3Rotation.dot(coordArray)
            data[1][0] = coordArray[0]
            data[1][1] = coordArray[1]
            data[1][2] = coordArray[2]

        for data in earthData:
            coordArray = np.array([data[1][0], data[1][1], data[1][2]]).reshape(3, 1)
            if data[3] == 1:
                coordArray = cam1Rotation.dot(coordArray)
            elif data[3] == 2:
                coordArray = cam2Rotation.dot(coordArray)
            elif data[3] == 3:
                coordArray = cam3Rotation.dot(coordArray)
            data[1][0] = coordArray[0]
            data[1][1] = coordArray[1]
            data[1][2] = coordArray[2]

        for data in moonData:
            coordArray = np.array([data[1][0], data[1][1], data[1][2]]).reshape(3, 1)
            if data[3] == 1:
                coordArray = cam1Rotation.dot(coordArray)
            elif data[3] == 2:
                coordArray = cam2Rotation.dot(coordArray)
            elif data[3] == 3:
                coordArray = cam3Rotation.dot(coordArray)
            data[1][0] = coordArray[0]
            data[1][1] = coordArray[1]
            data[1][2] = coordArray[2]

    return sunData, earthData, moonData

# rotate sun, moon, earth vectors so that they are at t=0
# here timestamp is now at time of first recording (should we use the rtc or pi's clock so that we can compare each observe sequence's time with eachother?



# Returns: timestamp, coordinate, radius for each


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-m", "--mode", help="Restart mode for camera mux or regular run")
    args = vars(ap.parse_args())

    if args["mode"] == "restart":
        mux = camera.CameraMux()
        mux.selectCamera(1)
        print("selected mux")
    else:
        warnings.filterwarnings("ignore")
        sun, earth, moon = observe()
        print("Sun:")
        print(sun)
        print("Earth:")
        print(earth)
        print("Moon:")
        print(moon)








