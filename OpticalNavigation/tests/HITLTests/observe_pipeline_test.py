import argparse
import os
import cv2
import re
import math

import core.img_preprocess as ip
import core.find
import core.camera

#import img_preprocess as ip
#import find
# import camera

# Adapted from https://www.geeksforgeeks.org/extract-images-from-video-in-python/
def mjpegToJpeg(inputFile):
    src = cv2.VideoCapture(inputFile)
    currentFrame = 0
    base = os.path.splitext(inputFile)[0]
    frames = []
    while (True):
        # Write next frame to a new jpeg image
        ret, frame = src.read()
        if ret:
            name = base + f"_f{currentFrame}.jpeg"  # Change filename, format string
            cv2.imwrite(name, frame)
            frames.append(name)
            currentFrame += 1
        else:
            break
    src.release()
    cv2.destroyAllWindows()
    return frames


def observe():
    # Capture --> preprocess --> find
    # Steps:
    # Record camera 1 exposure 1
    # Record camera 1 exposure 2
    # Record camera 2 exposure 1
    # Record camera 2 exposure 2
    # Record camera 3 exposure 1
    # Record camera 3 exposure 2
    #
    # Separate each video into images
    # Preprocess all six image sets
    #
    # Send each image set into find module
    #
    # Determine best image from image set
    # Return timestamp, coordinate, radius of sun, earth, moon on each of three cameras
    #
    # File naming convention:
    # Video: camera# + exposure.mjpeg
    # Image: camera# + exposure + frame#.jpeg
    #
    #

    recordings = {}

    # To use on flight hardware
    # mux = camera.CameraMux()
    # for i in [1, 2, 3]: # make sure that these numbers correspond ot the ports on the hardware
    #	mux.selectCamera(i)
    #	cam = camera.Camera()
    #	# Records exposure sequence 1 - change filename and shutterSpeed value
    #	file1, timestamp1 = cam.camera.rawObservation(f"cam{i}_expHigh.mjpeg", shutterSpeed = 50000 ) # add shutterspeed value
    #	# Records exposure sequence 2
    #	file2, timestamp2 = cam.camera.rawObservation(f"cam{i}_expLow.mjpeg", shutterSpeed = 20000 )
    #	recordings[file1] = timestamp1
    #	recordings[file2] = timestamp2

    print("Data retrieved from cameras:")
    print(recordings)

    # Now recordings has list of filename, last_timestamp associations
    # Artificially manipulate recordings to use space videos
    recordings["cam1_expHigh_rep.mjpeg"] = 10  # Sun video
    recordings["cam1_expLow_rep.mjpeg"] = 10  # Sun video
    recordings["cam2_expHigh_rep.mjpeg"] = 10  # Earth video
    recordings["cam2_expLow_rep.mjpeg"] = 10  # Earth video
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

    for filename, timestamp in recordings.items():
        frames = mjpegToJpeg(filename)
        # imgData = [sun, earth, moon, image, framename]
        # where celestial body = [x, y, radius]
        imgData = []
        # Extract find data for each frame, put into imgData[]
        for i in frames:
            ip.preprocess(i, i)
            img = cv2.imread(i)
            s, e, m, img = find.find(img, visualize=True)
            data = [s, e, m, img, i]
            imgData.append(data)

        centerX = 320  # Need to maintain with camera parameters
        centerY = 240  #

        exposureHigh = "High" in filename
        if exposureHigh:
            sunDist = 1000000  # Some large number
            sunImage = None
            for i in imgData:
                if i[0] is not None:
                    sunXdiff = abs(i[0][0] - centerX)
                    sunYdiff = abs(i[0][1] - centerY)
                    distToCenter = math.sqrt((sunXdiff ** 2) + (sunYdiff ** 2))
                    if distToCenter < sunDist:
                        sunDist = distToCenter
                        sunImage = i;

            if sunImage is not None:
                sunFrameNum = int(re.search("[f](\d+)", sunImage[4]).group(1))
                sunFrameTimestamp = timestamp - (len(frames) - 1 - sunFrameNum) * (1 / 1)  # FPS is denominator

                sunDataGroup = []
                sunDataGroup.append(sunFrameTimestamp)  # Timestamp
                sunDataGroup.append(sunImage[0][0:2])  # Coordinate
                sunDataGroup.append(sunImage[0][2])  # Radius
                sunDataGroup.append(sunImage[4])  # Filename for debugging
                sunData.append(sunDataGroup)

        # ----------------Done with sun processing---------------------------

        exposureLow = "Low" in filename
        if exposureLow:  ###is low exposure (earth/moon)
            earthDist = 1000000  # Some large number
            moonDist = 1000000  # Some large number
            earthImage = None
            moonImage = None
            for i in imgData:
                if i[1] is not None:
                    earthXdiff = abs(i[1][0] - centerX)
                    earthYdiff = abs(i[1][1] - centerY)
                    earthDistToCenter = math.sqrt((earthXdiff ** 2) + (earthYdiff ** 2))
                    if earthDistToCenter < earthDist:
                        earthDist = earthDistToCenter
                        earthImage = i;

                if i[2] is not None:
                    moonXdiff = abs(i[2][0] - centerX)
                    moonYdiff = abs(i[2][1] - centerY)
                    moonDistToCenter = math.sqrt((moonXdiff ** 2) + (moonYdiff ** 2))
                    if moonDistToCenter < moonDist:
                        moonDist = moonDistToCenter
                        moonImage = i;

            # FIX TIMESTAMP EQUATION!!!!

            if earthImage is not None:
                earthFrameNum = int(re.search("[f](\d+)", earthImage[4]).group(1))
                earthFrameTimestamp = timestamp - (len(frames) - 1 - earthFrameNum) * (1 / 1)  # FPS is denominator

                earthDataGroup = []
                earthDataGroup.append(earthFrameTimestamp)  # Timestamp
                earthDataGroup.append(earthImage[1][0:2])  # Coordinate
                earthDataGroup.append(earthImage[1][2])  # Radius
                earthDataGroup.append(earthImage[4])  # Filename for debugging
                earthData.append(earthDataGroup)

            if moonImage is not None:
                moonFrameNum = int(re.search("[f](\d+)", moonImage[4]).group(1))
                moonFrameTimestamp = timestamp - (len(frames) - 1 - earthFrameNum) * (1 / 1)  # FPS is denominator

                moonDataGroup = []
                moonDataGroup.append(moonFrameTimestamp)  # Timestamp
                moonDataGroup.append(moonImage[0][0:2])  # Coordinate
                moonDataGroup.append(moonImage[0][2])  # Radius
                moonDataGroup.append(moonImage[4])  # Filename for debugging
                moonData.append(moonDataGroup)

    return sunData, earthData, moonData


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
        sun, earth, moon = observe()
        print("Sun:")
        print(sun)
        print("Earth:")
        print(earth)
        print("Moon:")
        print(moon)









