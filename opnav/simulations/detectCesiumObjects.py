import numpy as np

# import scipy as sp
import cv2
import argparse
import copy
import os
import re
import pandas as pd
from tqdm import tqdm
import threading

from opnav.core.find_algos.find import round_up_to_odd
from opnav.core.preprocess import rectilinearToStereographicProjection

"""
Extracts circles from iteration images generated in Cesium for true measurements (red moon texture, no directional lighting)
"""

"""
Use this function for cropping the template sun image
to the desired size.
Returns a centered cropped sun.
"""


def fitCircleAroundReplacement():
    x = 583
    y = 421
    truer = 6
    r = truer * 60
    path = "C:\\Users\\easha\\Downloads\\UnityTemplateSun.PNG"
    image = cv2.imread(path)
    # cv2.circle(image, (x, y), r, (255, 255, 255), 2)
    # cv2.circle(image, (x, y), 2, (0, 0, 255), 1)
    # cv2.imshow('fit', image[y-r:y+r,x-r:x+r])
    # uncomment to crop sun image
    # cv2.imwrite(path, image[y-r:y+r,x-r:x+r])
    # cv2.waitKey(0)
    return image[y - r : y + r, x - r : x + r]


"""
Use this function for detecting the yellow Cesium Sun and replacing it with a new Sun image.
Precondition: Cesium Sun is generated using the following settings
in GenerateCesium.html:
scene.sun.glowFactor = 1;
scene.sun.sunBloom = true;
[image]: cv2 image where we are looking for the Sun
[path]: new Sun image to replace the current Sun image with
Returns:
image with new Sun, sun X and Y pos, sun radius (of new Sun)
"""


def detectAndReplaceCesiumSun(image, path):
    boundaries = [([10, 100, 20], [40, 255, 50])]
    # Replacement sun
    newSun = cv2.imread(path)
    assert newSun.shape[0] == newSun.shape[1]
    padSize = newSun.shape[0]
    image = np.pad(
        image,
        [(padSize, padSize), (padSize, padSize), (0, 0)],
        mode="constant",
        constant_values=0,
    )

    original_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    output = None
    # Note: Calculates mask for one boundary only
    for lower, upper in boundaries:
        lower = np.array(lower, dtype="uint8")
        upper = np.array(upper, dtype="uint8")

        mask = cv2.inRange(original_hsv, lower, upper)
        output = cv2.bitwise_and(original_hsv, original_hsv, mask=mask)

    gray = cv2.cvtColor(cv2.cvtColor(output, cv2.COLOR_HSV2BGR), cv2.COLOR_BGR2GRAY)
    # Increase gray mask brightness for non-black pixels only
    gray[gray > 1] = 255

    circles = cv2.HoughCircles(
        gray, cv2.HOUGH_GRADIENT, 2, 50, param1=80, param2=15, minRadius=1, maxRadius=0
    )
    if circles is not None:
        circle = circles[0][0]
        print(circle)
        x = int(circle[0])
        y = int(circle[1])
        r = int(circle[2])
        # Clear out current sun pixels
        image[y - y : y + r, x - r : x + r] = 0
        # Plot new sun
        newRad = int(padSize / 2)
        print(y - newRad, y + newRad, x - newRad, x + newRad)
        image[y - newRad : y + newRad, x - newRad : x + newRad] = newSun
        # cv2.circle(image, (x, y), r, (255, 255, 255), 2)
        # cv2.circle(image, (x, y), 2, (0, 0, 255), 1)
        return image[padSize:-padSize, padSize:-padSize, :], x - padSize, y - padSize, 6
    return None


"""
Use this function for detecting the yellow Cesium Sun and replacing it with a new Sun image.
Precondition: Cesium Sun is generated using the following settings
in GenerateCesium.html:
scene.sun.glowFactor = 1;
scene.sun.sunBloom = true;
[image]: cv2 image where we are looking for the Sun
[path]: new Sun image to replace the current Sun image with
Returns:
image with new Sun, sun X and Y pos, sun radius (of new Sun)
"""


def detectCesiumSun(image):
    boundaries = [([10, 100, 20], [40, 255, 50])]
    # Replacement sun
    padSize = int(max(image.shape[0] / 2, image.shape[1] / 2))
    image = np.pad(
        image,
        [(padSize, padSize), (padSize, padSize), (0, 0)],
        mode="constant",
        constant_values=0,
    )

    original_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    output = None
    # Note: Calculates mask for one boundary only
    for lower, upper in boundaries:
        lower = np.array(lower, dtype="uint8")
        upper = np.array(upper, dtype="uint8")

        mask = cv2.inRange(original_hsv, lower, upper)
        output = cv2.bitwise_and(original_hsv, original_hsv, mask=mask)

    gray = cv2.cvtColor(cv2.cvtColor(output, cv2.COLOR_HSV2BGR), cv2.COLOR_BGR2GRAY)
    # Increase gray mask brightness for non-black pixels only
    gray[gray > 1] = 255

    circles = cv2.HoughCircles(
        gray, cv2.HOUGH_GRADIENT, 2, 50, param1=80, param2=15, minRadius=1, maxRadius=0
    )
    if circles is not None:
        circle = circles[0][0]
        x = int(circle[0])
        y = int(circle[1])
        r = int(circle[2])
        # Clear out current sun pixels
        # image[y-y:y+r, x-r:x+r] = 0
        # Plot new sun
        # newRad = int(padSize/2)
        # print(y-newRad,y+newRad, x-newRad,x+newRad)
        # image[y-newRad:y+newRad, x-newRad:x+newRad] = newSun
        # cv2.circle(image, (x, y), r, (255, 255, 255), 2)
        # cv2.circle(image, (x, y), 2, (0, 0, 255), 1)
        return image[padSize:-padSize, padSize:-padSize, :], x - padSize, y - padSize, r
    return None


"""
Use this function for detecting the Cesium Earth.
Precondition: Cesium Moon is generated using the following settings
in GenerateCesium.html:
var isSimulatingForGroundTruthMeasurements = true;
...
var globe = scene.globe;
globe.lightingFadeOutDistance = 0;
globe.lightingFadeInDistance = 0;
globe.nightFadeOutDistance = 0;
globe.nightFadeInDistance = 1;
[image]: cv2 image where we are looking for the Earth
Returns the rescaled coordinates and radius as well as an image of a circle placed over the body
"""


def detectCesiumEarth(image):
    boundaries = [([80, 30, 1], [160, 254, 254])]
    # Replacement sun
    upsampledImage, upsampleFactor = cv2.pyrUp(cv2.pyrUp(cv2.pyrUp(image))), 2 * 2 * 2

    padSize = int(max(upsampledImage.shape[0] / 2, upsampledImage.shape[1] / 2))
    upsampledImage = np.pad(
        upsampledImage,
        [(padSize, padSize), (padSize, padSize), (0, 0)],
        mode="constant",
        constant_values=0,
    )

    original_hsv = cv2.cvtColor(upsampledImage, cv2.COLOR_BGR2HSV)

    output = None
    mask = None
    # Note: Calculates mask for one boundary only
    for lower, upper in boundaries:
        lower = np.array(lower, dtype="uint8")
        upper = np.array(upper, dtype="uint8")
        if mask is None:
            mask = cv2.inRange(original_hsv, lower, upper)
        else:
            mask += cv2.inRange(original_hsv, lower, upper)
        output = cv2.bitwise_and(original_hsv, original_hsv, mask=mask)

    gray = cv2.cvtColor(cv2.cvtColor(output, cv2.COLOR_HSV2BGR), cv2.COLOR_BGR2GRAY)

    # Increase gray mask brightness for non-black pixels only
    gray[gray > 1] = 255

    original_median_blurred = cv2.medianBlur(gray, round_up_to_odd(1))

    up_pad_circles = cv2.HoughCircles(
        original_median_blurred,
        cv2.HOUGH_GRADIENT,
        2,
        50,
        param1=80,
        param2=30,
        minRadius=1,
        maxRadius=0,
    )
    if up_pad_circles is not None:
        up_pad_circle = up_pad_circles[0][0]
        x = int(
            (up_pad_circle[0] - padSize) / upsampleFactor
        )  # The detected circle is always a bit off-center
        y = int((up_pad_circle[1] - padSize) / upsampleFactor)
        r = int((up_pad_circle[2]) / upsampleFactor)
        # r = int(r * 1.5) # The detected circle is always a bit smaller than is should be
        # x1 = int(up_pad_circle[0])
        # y1 = int(up_pad_circle[1])
        # r1 = int(up_pad_circle[2])
        cv2.circle(image, (x, y), r, (255, 0, 0), 2)
        cv2.circle(image, (x, y), 2, (0, 0, 255), 1)
        # uph, upw, _ = upsampledImage.shape
        return image, x, y, r
    return None


"""
Use this function for detecting the Cesium Moon with red texture.
Precondition: Cesium Moon is generated using the following settings
in GenerateCesium.html:
var isSimulatingForGroundTruthMeasurements = true;
[image]: cv2 image where we are looking for the red Moon
Returns the rescaled coordinates and radius as well as an image of a circle placed over the body
"""


def detectCesiumRedMoon(image):
    boundaries = [([0, 100, 100], [10, 255, 255]), ([160, 100, 100], [179, 255, 255])]
    # Replacement sun
    upsampledImage, upsampleFactor = cv2.pyrUp(cv2.pyrUp(cv2.pyrUp(image))), 2 * 2 * 2

    padSize = int(max(upsampledImage.shape[0] / 2, upsampledImage.shape[1] / 2))
    upsampledImage = np.pad(
        upsampledImage,
        [(padSize, padSize), (padSize, padSize), (0, 0)],
        mode="constant",
        constant_values=0,
    )

    original_hsv = cv2.cvtColor(upsampledImage, cv2.COLOR_BGR2HSV)

    output = None
    mask = None
    # Note: Calculates mask for one boundary only
    for lower, upper in boundaries:
        lower = np.array(lower, dtype="uint8")
        upper = np.array(upper, dtype="uint8")
        if mask is None:
            mask = cv2.inRange(original_hsv, lower, upper)
        else:
            mask += cv2.inRange(original_hsv, lower, upper)
        output = cv2.bitwise_and(original_hsv, original_hsv, mask=mask)

    gray = cv2.cvtColor(cv2.cvtColor(output, cv2.COLOR_HSV2BGR), cv2.COLOR_BGR2GRAY)

    gray_blurred = cv2.GaussianBlur(gray, (25, 25), 0)

    # Increase gray mask brightness for non-black pixels only
    # This should circularize the Moon
    gray_blurred[gray_blurred > 1] = 255

    original_median_blurred = cv2.medianBlur(gray_blurred, round_up_to_odd(11))

    up_pad_circles = cv2.HoughCircles(
        original_median_blurred,
        cv2.HOUGH_GRADIENT,
        2,
        50,
        param1=80,
        param2=30,
        minRadius=1,
        maxRadius=0,
    )
    if up_pad_circles is not None:
        up_pad_circle = up_pad_circles[0][0]
        x = int(
            (up_pad_circle[0] - padSize) / upsampleFactor
        )  # The detected circle is always a bit off-center
        y = int((up_pad_circle[1] - padSize) / upsampleFactor)
        r = int((up_pad_circle[2]) / upsampleFactor)
        # r = int(r * 1.5) # The detected circle is always a bit smaller than is should be
        # x1 = int(up_pad_circle[0])
        # y1 = int(up_pad_circle[1])
        # r1 = int(up_pad_circle[2])
        cv2.circle(image, (x, y), r, (255, 0, 0), 2)
        cv2.circle(image, (x, y), 2, (0, 0, 255), 1)
        # uph, upw, _ = upsampledImage.shape
        return image, x, y, r
    return None


def obtainCesiumDetections(batch, path, circleCSV, startIter, endIter):
    """
    - Given a set of images organized by iterations, this function returns set of detections
    - removes blank images
    [batch]: batch name
    [path]: location of iterations folder
    [circleCSV]: location of CSV file containing circles for each image
    [startIter]: starting iteration
    [endIter]: ending iteration
    """
    circleDict = {
        "Iteration": [],
        "Camera": [],
        "View": [],
        "EX": [],
        "EY": [],
        "ER": [],
        "MX": [],
        "MY": [],
        "MR": [],
        "SX": [],
        "SY": [],
        "SR": [],
    }
    # Overwrite existing file
    df = pd.DataFrame.from_dict(circleDict)
    df.to_csv(circleCSV, index=False)
    del df

    print(f"Thread started {batch} {circleCSV}")

    filenamereg = re.compile(r"(\d+)-(\d+)-(\d+)-(\d+)-(\d+)T(\d+) (\d+) (\d+)Z.png")

    def findObjectsInCamera(cam, camPath, currIter):
        circleDict = pd.read_csv(circleCSV).to_dict("list")

        for filename in os.listdir(camPath):
            imagePath = os.path.join(camPath, filename)
            assert os.path.exists(imagePath)
            image = cv2.imread(imagePath)

            if np.min(image) == np.max(image) == 0:
                os.remove(imagePath)
                continue
            stereoImage = rectilinearToStereographicProjection(image)
            moonRes = detectCesiumRedMoon(copy.copy(stereoImage))
            sunRes = detectCesiumSun(copy.copy(stereoImage))
            earthRes = detectCesiumEarth(copy.copy(stereoImage))

            # Verify we are looking at the correct image
            mo = filenamereg.search(filename)
            assert mo is not None
            iteration = int(mo.groups()[0])
            assert iteration == currIter
            view = int(mo.groups()[1])
            camera = 1
            if view <= 15 and view >= 8:
                camera = 2

            elif view >= 16:
                camera = 3
            assert camera == cam

            del camera, mo, iteration

            circleDict["Iteration"].append(currIter)
            circleDict["Camera"].append(cam)
            circleDict["View"].append(view)
            moonX, moonY, moonR, earthX, earthY, earthR, sunX, sunY, sunR = (
                "-",
                "-",
                "-",
                "-",
                "-",
                "-",
                "-",
                "-",
                "-",
            )
            if moonRes is not None:
                (_, moonX, moonY, moonR) = moonRes
                cv2.circle(stereoImage, (moonX, moonY), moonR, (255, 255, 255), 2)
                cv2.circle(stereoImage, (moonX, moonY), 2, (0, 0, 255), 1)

            if earthRes is not None:
                (_, earthX, earthY, earthR) = earthRes
                cv2.circle(stereoImage, (earthX, earthY), earthR, (255, 0, 0), 2)
                cv2.circle(stereoImage, (earthX, earthY), 2, (0, 0, 255), 1)

            if sunRes is not None:
                (_, sunX, sunY, sunR) = sunRes
                cv2.circle(stereoImage, (sunX, sunY), sunR, (0, 255, 255), 2)
                cv2.circle(stereoImage, (sunX, sunY), 2, (0, 0, 255), 1)

            # cv2.imshow(f'Stereographic corrected {filename}', stereoImage)
            # cv2.waitKey(500)
            # cv2.destroyWindow(f'Stereographic corrected {filename}')

            # Dump data into csv
            circleDict["MX"].append(moonX)
            circleDict["MY"].append(moonY)
            circleDict["MR"].append(moonR)
            circleDict["EX"].append(earthX)
            circleDict["EY"].append(earthY)
            circleDict["ER"].append(earthR)
            circleDict["SX"].append(sunX)
            circleDict["SY"].append(sunY)
            circleDict["SR"].append(sunR)

        assert (
            len(circleDict["Iteration"])
            == len(circleDict["Camera"])
            == len(circleDict["View"])
            == len(circleDict["MX"])
            == len(circleDict["MY"])
            == len(circleDict["MR"])
            == len(circleDict["EX"])
            == len(circleDict["EY"])
            == len(circleDict["ER"])
            == len(circleDict["SX"])
            == len(circleDict["SY"])
            == len(circleDict["SR"])
        )
        df = pd.DataFrame.from_dict(circleDict)
        df.to_csv(circleCSV, index=False)
        del df

    for currIter in tqdm(range(startIter, endIter + 1), desc=f"[{batch}]"):
        iterPath = os.path.join(path, f"{currIter}")
        assert os.path.exists(iterPath)
        cam1Path = os.path.join(iterPath, f"1")
        cam2Path = os.path.join(iterPath, f"2")
        cam3Path = os.path.join(iterPath, f"3")
        # Check if all camera views exists
        if (
            not os.path.exists(cam1Path)
            or not os.path.exists(cam2Path)
            or not os.path.exists(cam3Path)
        ):
            continue
        findObjectsInCamera(1, cam1Path, currIter)
        findObjectsInCamera(2, cam2Path, currIter)
        findObjectsInCamera(3, cam3Path, currIter)

    print(f"Thread ended {batch} {circleCSV}")


def combineCirclesCSVs(circleCSVDir):
    """
    [circleCSVDir]: path to circles#.csv directory
    Combines all circles.csv
    """
    iter = 0
    circlePath = os.path.join(circleCSVDir, f"circles{iter}.csv")
    circleDf = None
    while os.path.exists(circlePath):
        if circleDf is None:
            circleDf = pd.read_csv(circlePath)
        else:
            tempDf = pd.read_csv(circlePath)
            circleDf = pd.concat([circleDf, tempDf])
        os.remove(circlePath)
        iter += 1
        circlePath = os.path.join(circleCSVDir, f"circles{iter}.csv")
        print(iter)
    if circleDf is not None:
        circleDf.to_csv(os.path.join(circleCSVDir, f"circlesCombined.csv"), index=False)


class CesiumDetectionThread(threading.Thread):
    def __init__(self, threadID, batch, path, circleCSV, startIter, endIter):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.batch = batch
        self.path = path
        self.circleCSV = circleCSV
        self.startIter = startIter
        self.endIter = endIter

    def run(self):
        print("Starting " + self.batch)
        obtainCesiumDetections(
            self.batch, self.path, self.circleCSV, self.startIter, self.endIter
        )
        print("Exiting " + self.batch)


if __name__ == "__main__":
    """
    Run "python FilterCesiumSun.py -i=<IMAGE>" to test this module
    """
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--path", help="path to the iterations folder")
    ap.add_argument("-c", "--circlepath", help="path to the circles folder")
    ap.add_argument("-s", "--startiter", help="starting iteration (folder name)")
    ap.add_argument("-e", "--enditer", help="ending iteration (folder name)")
    ap.add_argument(
        "-b",
        "--batchsize",
        help="number of iterations per batch for parallel computing",
    )
    args = vars(ap.parse_args())

    batchSize = int(args["batchsize"])
    startIter = int(args["startiter"])
    endIter = int(args["enditer"])
    assert (
        batchSize >= 1
        and startIter <= endIter
        and batchSize <= (endIter - startIter + 1)
    )
    l = np.arange(startIter, endIter, batchSize)
    threads = []
    for index, ite in enumerate(l):
        start = ite
        end = min(ite + batchSize - 1, endIter)
        thread = CesiumDetectionThread(
            index,
            f"Batch-{index}",
            args["path"],
            os.path.join(args["circlepath"], f"circles{index}.csv"),
            start,
            end,
        )
        thread.start()
        threads.append(thread)

    for t in threads:
        t.join()

    combineCirclesCSVs(args["circlepath"])

    # Render single image
    # TEMPLATE_SUN_PATH = "C:\\Users\\easha\\Downloads\\UnityTemplateSun.PNG"

    # image = cv2.imread(args["path"])
    # stereoImage = rectilinearToStereographicProjection(image)
    # moonRes = detectCesiumRedMoon(copy.copy(stereoImage))
    # sunRes = detectCesiumSun(copy.copy(stereoImage))
    # earthRes = detectCesiumEarth(copy.copy(stereoImage))
    # if moonRes is not None:
    #     (_, moonX, moonY, moonR) = moonRes
    #     print(f'MOON: {moonX, moonY, moonR}')
    #     cv2.circle(stereoImage, (moonX, moonY), moonR, (255, 255, 255), 2)
    #     cv2.circle(stereoImage, (moonX, moonY), 2, (0, 0, 255), 1)

    # if earthRes is not None:
    #     (_, earthX, earthY, earthR) = earthRes
    #     print(f'EARTH: {earthX, earthY, earthR}')
    #     cv2.circle(stereoImage, (earthX, earthY), earthR, (255, 0, 0), 2)
    #     cv2.circle(stereoImage, (earthX, earthY), 2, (0, 0, 255), 1)

    # if sunRes is not None:
    #     (_, sunX, sunY, sunR) = sunRes
    #     print(f'SUN: {sunX, sunY, sunR}')
    #     cv2.circle(stereoImage, (sunX, sunY), sunR, (0, 255, 255), 2)
    #     cv2.circle(stereoImage, (sunX, sunY), 2, (0, 0, 255), 1)
    # if stereoImage is not None:
    #     cv2.imshow("Result Stereographic", stereoImage)
    #     cv2.waitKey(0)
    # else:
    #     print("NO circle")
