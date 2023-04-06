from fsw.opnav.core.find_algos.find import find, round_up_to_odd, create_circular_mask
from fsw.opnav.tests.test_find import calculateErrors
import cv2
import os
import glob
import numpy as np
import copy
import pandas as pd
import itertools
import argparse
from tqdm import tqdm
import time

"""
This file contains code for testing the detector module by
generating fake configurations of the Sun, Moon and Earth using
real images. The goal is to obtain optimal Hough Transform
parameters such that the detector performs within a reasonable margin
of error in any situation (since we don't know how the true images
will look like).

Folder Structure:
-Root/
    -Earth/
        -Crescent/
            -<images>
        -Half/
            -<images>
        -Ellipse/
            -<images>
        -Full/
            -<images>
    -Moon/
        -Crescent/
            -<images>
        -Half/
            -<images>
        -Ellipse/
            -<images>
        -Full/
            -<images>
    -Sun/
        -<images>
    -metadata.csv

The metadata.csv file contains the correct center and size of
each object in all the images. This information will be used
to crop, resize and rotate the objects in the generated images
such that their center and size can be easily obtained. The new
center and size will form the ground truth against which the
detector module will be tested.
"""

DETECTOR_DATASET_GENERATOR_PATH = "D:\\OpNav\\data\\DetectorDatasetGenerator\\"

EARTH_TEMPLATES_SUBFOLDER = os.path.join(DETECTOR_DATASET_GENERATOR_PATH, "Earth")
MOON_TEMPLATES_SUBFOLDER = os.path.join(DETECTOR_DATASET_GENERATOR_PATH, "Moon")
SUN_TEMPLATES_SUBFOLDER = os.path.join(DETECTOR_DATASET_GENERATOR_PATH, "Sun")
METADATA_CSV = os.path.join(DETECTOR_DATASET_GENERATOR_PATH, "metadata.csv")

CRESCENT_TEMPLATES_SUBFOLDER_NAME = "Crescent"
HALF_TEMPLATES_SUBFOLDER_NAME = "Half"
ELLIPSE_TEMPLATES_SUBFOLDER_NAME = "Ellipse"
FULL_TEMPLATES_SUBFOLDER_NAME = "Full"

ENTER_KEY = 13
SPACE_KEY = 32

# Parse dataset


def parse():
    """
    Render all images in a sequence to obtain the Earth's, Sun's and Moon's
    frame. It will be stored in metadata.csv.

    Warning: This will overwrite the metadata.csv.
    """
    # ALl folders
    # .jpg, .JPG, .jpeg, .png
    allFolders = [
        os.path.join(EARTH_TEMPLATES_SUBFOLDER, CRESCENT_TEMPLATES_SUBFOLDER_NAME),
        os.path.join(EARTH_TEMPLATES_SUBFOLDER, HALF_TEMPLATES_SUBFOLDER_NAME),
        os.path.join(EARTH_TEMPLATES_SUBFOLDER, ELLIPSE_TEMPLATES_SUBFOLDER_NAME),
        os.path.join(EARTH_TEMPLATES_SUBFOLDER, FULL_TEMPLATES_SUBFOLDER_NAME),
        os.path.join(MOON_TEMPLATES_SUBFOLDER, CRESCENT_TEMPLATES_SUBFOLDER_NAME),
        os.path.join(MOON_TEMPLATES_SUBFOLDER, HALF_TEMPLATES_SUBFOLDER_NAME),
        os.path.join(MOON_TEMPLATES_SUBFOLDER, ELLIPSE_TEMPLATES_SUBFOLDER_NAME),
        os.path.join(MOON_TEMPLATES_SUBFOLDER, FULL_TEMPLATES_SUBFOLDER_NAME),
        SUN_TEMPLATES_SUBFOLDER,
    ]
    names = []
    x = []
    y = []
    s = []
    for folder in allFolders:
        types = (
            os.path.join(folder, "*.jpg"),
            os.path.join(folder, "*.JPG"),
            os.path.join(folder, "*.png"),
            os.path.join(folder, "*.jpeg"),
        )
        files = []
        for extension in types:
            files.extend(glob.glob(extension))
        if len(files) == 0:
            if not os.path.isdir(folder):
                raise Exception(f"Incorrect directory: {folder}")
            else:
                raise Exception(f"No images found in {folder}")
        for i, file in enumerate(files):
            if file in names:
                # avoid duplicates
                continue

            original = cv2.imread(file)
            original_gray = cv2.cvtColor(original, cv2.COLOR_BGR2GRAY)
            scale = 1
            (c, r) = np.shape(original_gray)
            rows = int(np.round(r * scale))
            cols = int(np.round(c * scale))
            original_median_blurred = cv2.medianBlur(original_gray, round_up_to_odd(rows / 50))
            circles = cv2.HoughCircles(
                original_median_blurred,
                cv2.HOUGH_GRADIENT,
                1,
                150,
                param1=200,
                param2=20,
                minRadius=0,
                maxRadius=0,
            )
            original_with_circles = copy.copy(original)
            if circles is None:
                print("No Circles Detected!")
                names.append(file)
                x.append("-")
                y.append("-")
                s.append("-")
            else:
                for i in circles[0, :]:
                    cv2.circle(original_with_circles, (i[0], i[1]), i[2], (0, 255, 0), 2)
                    cv2.circle(original_with_circles, (i[0], i[1]), 2, (0, 0, 255), 3)
                    cv2.imshow(f"{file}", original_with_circles)
                    k = cv2.waitKey(0)
                    cv2.destroyAllWindows()
                    if k == ENTER_KEY:
                        # Accept current circle
                        names.append(file)
                        x.append(i[0])
                        y.append(i[1])
                        s.append(i[2])
                        break
                    elif k == SPACE_KEY:
                        # Reject current circle but keep exploring
                        print("Showing next.")
    df = pd.DataFrame.from_dict({"Name": names, "X": x, "Y": y, "S": s})
    df.to_csv(METADATA_CSV)


def cropImage(name, x, y, s):
    """
    Extracts the object from image and centers it on screen
    """
    x = int(x)
    y = int(y)
    s = int(s)
    original = cv2.imread(name)
    (r, c, d) = np.shape(original)
    if SUN_TEMPLATES_SUBFOLDER in name:
        selectionSize = int(min(c - x, x, r - y, y))
        selection = original[
            y - selectionSize: y + selectionSize,
            x - selectionSize: x + selectionSize,
            :,
        ]
        img = selection
    else:
        img = np.zeros((s * 2, s * 2, 3), np.uint8)
        left = max(x - s, 0)
        right = min(x + s, c)
        top = max(y - s, 0)
        bottom = min(y + s, r)
        selection = original[top:bottom, left:right, :]
        (nr, nc, nd) = np.shape(selection)
        newX = s
        newY = s
        newS = s
        # how far away is pseudo center from new center?
        left_padding = left - x + s
        right_padding = x - right + s
        top_padding = top - y + s
        bottom_padding = y - bottom + s
        img = np.pad(
            selection,
            [(top_padding, bottom_padding), (left_padding, right_padding), (0, 0)],
            mode="constant",
            constant_values=0,
        )
    return img


def showMetaSelections():
    """
    Display images and their circles
    """

    df = pd.read_csv(METADATA_CSV)
    for index, row in df.iterrows():
        name = str(row["Name"])
        x = int(row["X"])
        y = int(row["Y"])
        s = int(row["S"])
        img = cropImage(name, x, y, s)
        (r, c, d) = img.shape
        cx = int(r / 2)
        cy = int(c / 2)
        cv2.circle(img, (cx, cy), s, (0, 255, 0), 2)
        cv2.circle(img, (cx, cy), 2, (0, 0, 255), 3)
        cv2.imshow("Show", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


def rotate(image, angle, center=None):
    (h, w) = image.shape[:2]

    if center is None:
        center = (w / 2, h / 2)

    # Perform the rotation
    M = cv2.getRotationMatrix2D(center, angle, 1.0)
    rotated = cv2.warpAffine(image, M, (w, h))

    return rotated


class ImageConfiguration:
    def __init__(self):
        self.earth, self.moon, self.sun = None, None, None
        self.earthScale, self.moonScale, self.sunScale = None, None, None
        self.earthRotation, self.moonRotation, self.sunRotation = None, None, None
        self.earthBlur, self.moonBlur, self.sunBlur = None, None, None
        self.earthCircle, self.moonCircle, self.sunCircle = [], [], []
        self.earthAB, self.moonAB, self.sunAB = [0, 0], [0, 0], [0, 0]
        self.background = None
        self.imageNoise = None

    def addEarth(self, name, x, y, s, pos, size, rotation, blur, alpha, beta):
        """
        [name]: path to image
        [x], [y], [s]: metadata of the Earth taken from metadata.csv (center x, center y and radius)
        [pos]: pixel position in new image
        [size]: pixel radius in new image
        [rotation]: degrees of rotation
        [blur]: ...
        [alpha]: gain (contrast), float, must be greater than 0
        [beta]: bias (brightness), float
        """
        self.earth = cropImage(name, x, y, s)
        self.earthMask = create_circular_mask(
            self.earth.shape[0],
            self.earth.shape[1],
            center=np.array([pos[0], pos[1]]),
            radius=np.array([size]),
        )
        self.earthScale = size / s
        self.earthRotation = rotation
        self.earthBlur = blur
        self.earthCircle = [pos[0], pos[1], size]
        self.earthAB = [alpha, beta]

    def addMoon(self, name, x, y, s, pos, size, rotation, blur, alpha, beta):
        self.moon = cropImage(name, x, y, s)
        self.moonMask = create_circular_mask(
            self.moon.shape[0],
            self.moon.shape[1],
            center=np.array([pos[0], pos[1]]),
            radius=np.array([size]),
        )
        self.moonScale = size / s
        self.moonRotation = rotation
        self.moonBlur = blur
        self.moonCircle = [pos[0], pos[1], size]
        self.moonAB = [alpha, beta]

    def addSun(self, name, x, y, s, pos, size, rotation, blur, alpha, beta):
        self.sun = cropImage(name, x, y, s)
        self.sunScale = size / s
        self.sunRotation = rotation
        self.sunBlur = blur
        self.sunCircle = [pos[0], pos[1], s]
        self.sunAB = [alpha, beta]

    def applyTransformations(self, circle, body, mask, ab, rotation, blur):
        dim = (circle[2] * 2, circle[2] * 2)
        resized = cv2.resize(body, dim, interpolation=cv2.INTER_AREA)
        if mask is None:
            resized = np.clip(resized * ab[0] + ab[1], a_min=0, a_max=255)
        else:
            mask = create_circular_mask(
                resized.shape[0],
                resized.shape[1],
                center=np.array([circle[0], circle[1]]),
                radius=np.array([circle[2]]),
            )
            resized[mask] = np.clip(resized[mask] * ab[0] + ab[1], a_min=0, a_max=255)
        rotated = rotate(resized, rotation)
        if blur is not None:
            gausBlur = cv2.GaussianBlur(rotated, (blur, blur), 0)
            return gausBlur
        return rotated

    # def drawStars(self, image, countIntensity):
    #     self.stars = np.zeros((image.shape[0], image.shape[1]))
    #     (r,c, d) = image.shape
    #     for i in range(int(500*countIntensity)):
    #         pos = randint(0, r - 1), randint(0, c - 1)
    #         color = randint(200,255)
    #         self.stars[pos[0], pos[1]] = color
    #         image[pos[0], pos[1], 0] = color
    #         image[pos[0], pos[1], 1] = color
    #         image[pos[0], pos[1], 2] = color
    #     return image

    def draw(
        self,
        image,
        _left,
        _right,
        _top,
        _bottom,
        noise,
        drawCircles=False,
        applyBlurs=True,
    ):
        # self.left = _left
        # self.right = _right
        # self.top = _top
        # self.bottom = _bottom

        self.xCoord, self.yCoord, self.radPixels = None, None, None

        # image[_top:_bottom, _left:_right, :] = self.drawStars(image[_top:_bottom, _left:_right, :], countIntensity)

        if self.earth is not None:
            e = self.applyTransformations(
                self.earthCircle,
                self.earth,
                self.earthMask,
                self.earthAB,
                self.earthRotation,
                self.earthBlur,
            )
            left = int(self.earthCircle[1] - self.earthCircle[2])
            right = int(self.earthCircle[1] + self.earthCircle[2])
            top = int(self.earthCircle[0] - self.earthCircle[2])
            bottom = int(self.earthCircle[0] + self.earthCircle[2])
            image[left:right, top:bottom, :3] = e
            self.xCoord = (left + right) / 2
            self.yCoord = (top + bottom) / 2
            self.radPixels = self.earthCircle[2]

        if self.moon is not None:
            m = self.applyTransformations(
                self.moonCircle,
                self.moon,
                self.moonMask,
                self.moonAB,
                self.moonRotation,
                self.moonBlur,
            )
            left = int(self.moonCircle[1] - self.moonCircle[2])
            right = int(self.moonCircle[1] + self.moonCircle[2])
            top = int(self.moonCircle[0] - self.moonCircle[2])
            bottom = int(self.moonCircle[0] + self.moonCircle[2])
            image[left:right, top:bottom, :] += np.array(m, dtype=np.uint8)
            self.xCoord = (left + right) / 2
            self.yCoord = (top + bottom) / 2
            self.radPixels = self.moonCircle[2]
        # add noise
        # self.noise = np.array([randint(0,no)*2-40 for i in range(image.shape[0] * image.shape[1] * 3)],dtype=np.uint8).reshape(image.shape[0], image.shape[1], 3)
        # self.noise = np.random.normal(0, noiseIntensity, size=(image.shape[0], image.shape[1], 3))

        if self.sun is not None:
            sunC = self.sunCircle[2] * self.sunScale
            s = self.applyTransformations(
                [self.sunCircle[0], self.sunCircle[1], sunC],
                self.sun,
                None,
                self.sunAB,
                self.sunRotation,
                self.sunBlur,
            )
            left = int(self.sunCircle[1] - sunC)
            right = int(self.sunCircle[1] + sunC)
            top = int(self.sunCircle[0] - sunC)
            bottom = int(self.sunCircle[0] + sunC)
            image[left:right, top:bottom, :3] = s
            self.xCoord = (left + right) / 2
            self.yCoord = (top + bottom) / 2
            self.radPixels = self.sunCircle[2]

        self.noise = noise
        image = np.clip(image + self.noise, a_min=0, a_max=255)
        image = image.astype(np.uint8)

        # Apply small blur to entire image
        a = max(_bottom - _top, _right - _left)
        if applyBlurs:
            image = cv2.GaussianBlur(image, (round_up_to_odd(5), round_up_to_odd(5)), 0)
        if drawCircles and self.earth is not None:
            cv2.circle(
                image,
                (self.earthCircle[0], self.earthCircle[1]),
                self.earthCircle[2],
                (255, 0, 0),
                2,
            )

        if drawCircles and self.moon is not None:
            cv2.circle(
                image,
                (self.moonCircle[0], self.moonCircle[1]),
                self.moonCircle[2],
                (150, 150, 200),
                2,
            )

        if drawCircles and self.sun is not None:
            cv2.circle(
                image,
                (self.sunCircle[0], self.sunCircle[1]),
                self.sunCircle[2],
                (255, 255, 0),
                2,
            )

        # print(f'Before: {image.shape}')
        # image = image[_top:_bottom, _left:_right, :]
        # print(f'After: {image.shape}')
        # print(_top,_bottom, _left,_right,)

        return image, self.xCoord, self.yCoord, self.radPixels


def singular(objects, df, bodyType, radiuses, runFind=True):
    resultsdf = pd.DataFrame({"name": [], "true": [], "false": [], "centerErr": [], "radiusErr": []})

    resultsdf.to_csv(
        f"D:\\OpNav\\data\\DetectorDatasetGenerator\\{bodyType}_results.csv",
        index=False,
    )
    count = 0
    for name in tqdm(objects, desc="Object"):
        print(name)

        w, h = 1640, 1232

        row = df.loc[df["Name"] == name]

        configurations = []

        noiseSigmas = np.round(np.arange(0, 15, 2), 0)[::-1]  # 5
        noises = np.zeros((h, w, noiseSigmas.shape[0], 3))
        rotations = np.array([0])
        filename = os.path.join(
            "C:\\github\\FlightSoftware\\opnav\\experiments",
            f"GaussianNoises.npy",
        )
        blurs = np.array([1])
        contrasts = np.round(np.arange(0.5, 3.1, 0.5), 1)  # 6
        brightnesses = np.arange(-50, 51, 20)  # 7

        # For generating noise
        # with open(filename, 'wb') as f:
        #     for i, sigma in enumerate(noiseSigmas):
        #         print(sigma)
        #         a = np.random.normal(0, sigma, size=(h, w, 3))
        #         print(f'{np.min(a), np.max(a)}')
        #         cv2.imshow(f'Noise with sigma={sigma}', a)
        #         cv2.waitKey(1000)
        #         cv2.destroyAllWindows()
        #         np.save(f, a)

        # exit()

        # For loading saved noises
        with open(filename, "rb") as f:
            for i, sigma in enumerate(noiseSigmas):
                noises[:, :, i, :] = np.load(f)
                print(f"{np.min(noises[:,:,i,:]), np.max(noises[:,:,i,:])}")
                # cv2.imshow(f'Noise with sigma={sigma}', noises[:,:,i,:])
                # cv2.waitKey(1000)
                # cv2.destroyAllWindows()

        for radius in radiuses:
            # xs = np.arange(radius,min(h,w)-radius*2, 400)
            # ys = np.arange(radius,min(h,w)-radius*2, 400)
            xs = np.array([int(h / 2) - radius])
            ys = np.array([int(w / 2) - radius])
            configurations.extend(
                [
                    [radius, i, j, k, l, m, n, p]
                    for i in list(xs)
                    for j in list(ys)
                    for k in list(rotations)
                    for l in list(blurs)
                    for m in list(contrasts)
                    for n in list(np.arange(noiseSigmas.shape[0]))
                    for p in list(brightnesses)
                ]
            )

        print(len(configurations))
        for conf in tqdm(configurations, desc="Configuration"):
            radius, x, y, rotation, blur, contrast, noiseSigmaIndex, brightness = (
                conf[0],
                conf[1],
                conf[2],
                conf[3],
                conf[4],
                conf[5],
                conf[6],
                conf[7],
            )
            noiseSigma = noiseSigmas[noiseSigmaIndex]
            tqdm.write(
                f"Radius: {radius}/x,y: {x,y}/rotation: {rotation}/contrast: {contrast}/noise sigma: {noiseSigma}/brightness: {brightness}"
            )

            pos = [x, y]

            imgName = (
                f"{count}_{bodyType}_{radius}_{x}_{y}_{contrast}_{noiseSigma}_{brightness}_{os.path.basename(name)}"
            )

            config = ImageConfiguration()
            if bodyType == "Earth":
                config.addEarth(
                    name,
                    row["X"],
                    row["Y"],
                    row["S"],
                    pos=pos,
                    size=radius,
                    rotation=rotation,
                    blur=blur,
                    alpha=contrast,
                    beta=brightness,
                )
            elif bodyType == "Moon":
                config.addMoon(
                    name,
                    row["X"],
                    row["Y"],
                    row["S"],
                    pos=pos,
                    size=radius,
                    rotation=rotation,
                    blur=blur,
                    alpha=contrast,
                    beta=brightness,
                )
            else:
                config.addSun(
                    name,
                    row["X"],
                    row["Y"],
                    row["S"],
                    pos=pos,
                    size=radius,
                    rotation=rotation,
                    blur=blur,
                    alpha=contrast,
                    beta=brightness,
                )

            canvas = np.zeros((h, w, 3), np.uint8)

            img, xCoord, yCoord, radPixels = config.draw(
                canvas,
                _left=0,
                _right=h,
                _top=0,
                _bottom=w,
                noise=noises[:, :, noiseSigmaIndex, :],
                applyBlurs=False,
                drawCircles=False,
            )

            startTime = time.time()
            # img = cv2.imread(os.path.join('D:\\OpNav\\data\\DetectorDatasetGenerator\\SmallSat20', imgName))

            sunCircle, earthCircle, moonCircle, imgCircles = find(img, visualize=False)
            # tqdm.write(f'{sunCircle}, {earthCircle}, {moonCircle}')
            tqdm.write(f"Elapsed time: {time.time()-startTime} seconds.")

            incorrect = False
            if bodyType == "Earth" and earthCircle is None:
                incorrect = True
            if bodyType == "Moon" and moonCircle is None:
                incorrect = True
            if bodyType == "Sun" and sunCircle is None:
                incorrect = True

            if bodyType == "Earth" and not incorrect:
                # tqdm.write(f'{earthCircle, [yCoord, xCoord, radPixels]}')
                centerDistance, radiusDistance = calculateErrors(earthCircle, [yCoord, xCoord, radPixels])
            if bodyType == "Moon" and not incorrect:
                # tqdm.write(f'{moonCircle, [yCoord, xCoord, radPixels]}')
                centerDistance, radiusDistance = calculateErrors(moonCircle, [yCoord, xCoord, radPixels])
            if bodyType == "Sun" and not incorrect:
                # tqdm.write(f'{sunCircle, [yCoord, xCoord, radPixels]}')
                centerDistance, radiusDistance = calculateErrors(sunCircle, [yCoord, xCoord, radPixels])

            # False positive
            if incorrect:
                nextFalse = 1
                nextTrue = 0
                centerErr = -1
                radiusErr = -1
            # True positive
            if not incorrect:
                tqdm.write(f"center error: {centerDistance}, radius error: {radiusDistance}")
                nextTrue = 1
                nextFalse = 0
                centerErr = np.round(centerDistance, 1)
                radiusErr = np.round(radiusDistance, 1)
            if incorrect or centerDistance > 20:
                if incorrect:
                    tqdm.write("Incorrect")
                    filename = f"Incorrect_{radius}_{x}_{y}_{rotation}_{blur}_{contrast}_{noiseSigma}_{brightness}_{os.path.basename(name)}"
                elif centerDistance > 20:
                    tqdm.write(f"Too off center: {centerDistance}")
                    filename = f"{round(centerDistance),2}_{round(radiusDistance,2)}_{radius}_{x}_{y}_{rotation}_{blur}_{contrast}_{noiseSigma}_{brightness}_{os.path.basename(name)}"

            count += 1

            # data dump
            # resultsdf = pd.read_csv(f'D:\\OpNav\\data\\DetectorDatasetGenerator\\{bodyType}_results.csv')
            d = {
                "name": imgName,
                "true": nextTrue,
                "false": nextFalse,
                "centerErr": centerErr,
                "radiusErr": radiusErr,
            }
            resultsdf = resultsdf.append([d], ignore_index=True)
            # resultsdf['name'].append(nextName)
            resultsdf.to_csv(
                f"D:\\OpNav\\data\\DetectorDatasetGenerator\\{bodyType}_results.csv",
                index=False,
            )
            cv2.imwrite(
                os.path.join("D:\\OpNav\\data\\DetectorDatasetGenerator\\SmallSat20", imgName),
                img,
            )
            del img, imgCircles


def three(earthPath, moonPath, sunPath):
    w, h = 1640, 1232
    df = pd.read_csv(METADATA_CSV)
    noiseSigmas = np.round(np.arange(0, 15, 2), 0)[::-1]  # 5
    noises = np.zeros((h, w, noiseSigmas.shape[0], 3))
    rotations = np.array([0])
    filename = os.path.join(
        "C:\\github\\FlightSoftware\\opnav\\experiments",
        f"GaussianNoises.npy",
    )
    blurs = np.array([1])
    contrasts = np.round(np.arange(0.5, 3.1, 0.5), 1)  # 6
    brightnesses = np.arange(-50, 51, 20)  # 7

    # radius, x, y, rotation, blur, contrast, noiseSigmaIndex, brightness = conf[0], conf[1], conf[2], conf[3], conf[4], conf[5], conf[6], conf[7]
    # earth conf
    earthConf = [30, 900, 1000, 0, 1, 1, 0]
    # moon conf
    moonConf = [5, 300, 300, 0, 1, 1, 0]
    # sun conf
    sunConf = [100, 1200, 600, 0, 1, 1, 0]
    config = ImageConfiguration()
    earthRow = df.loc[df["Name"] == earthPath]
    moonRow = df.loc[df["Name"] == moonPath]
    sunRow = df.loc[df["Name"] == sunPath]

    config.addEarth(
        earthPath,
        earthRow["X"],
        earthRow["Y"],
        earthRow["S"],
        pos=earthConf[1:3],
        size=earthConf[0],
        rotation=earthConf[3],
        blur=earthConf[4],
        alpha=earthConf[5],
        beta=earthConf[6],
    )
    config.addMoon(
        moonPath,
        moonRow["X"],
        moonRow["Y"],
        moonRow["S"],
        pos=moonConf[1:3],
        size=moonConf[0],
        rotation=moonConf[3],
        blur=moonConf[4],
        alpha=moonConf[5],
        beta=moonConf[6],
    )
    # config.addSun(sunPath, sunRow['X'], sunRow['Y'], sunRow['S'], pos=sunConf[1:3], size=sunConf[0], rotation=sunConf[3], blur=sunConf[4], alpha=sunConf[5], beta=sunConf[6])

    canvas = np.zeros((h, w, 3), np.uint8)

    img, xCoord, yCoord, radPixels = config.draw(
        canvas,
        _left=0,
        _right=h,
        _top=0,
        _bottom=w,
        noise=noises[:, :, noiseSigmas.shape[0] - 1, :],
        applyBlurs=True,
        drawCircles=False,
    )

    cv2.imshow("All three", img)
    cv2.waitKey(0)
    cv2.imwrite("C:\\github\\FlightSoftware\\opnav\\experiments\\Image.jpg", img)


def generateSingleImages(bodyType):
    """
    Generates images with randomness. Each image represents
    a random configuration of the following variables:
    - # of bodies present {[Earth], [Sun], [Moon], [Earth,Sun], [Earth,Moon], [Moon,Sun], [Earth,Sun,Moon]}
    - Body position {area inside image, eclipse event}
    - Body scale
    - Body rotation
    - Body blur {Gaussian, stretching in a direction}
    - Body brightness/contrast: https://docs.opencv.org/2.4/doc/tutorials/core/basic_linear_transform/basic_linear_transform.html
    - Background {Stars, No Stars}
    - Image Noise {RGB noise}
    """
    allFolders = [
        os.path.join(EARTH_TEMPLATES_SUBFOLDER, CRESCENT_TEMPLATES_SUBFOLDER_NAME),
        os.path.join(EARTH_TEMPLATES_SUBFOLDER, HALF_TEMPLATES_SUBFOLDER_NAME),
        os.path.join(EARTH_TEMPLATES_SUBFOLDER, ELLIPSE_TEMPLATES_SUBFOLDER_NAME),
        os.path.join(EARTH_TEMPLATES_SUBFOLDER, FULL_TEMPLATES_SUBFOLDER_NAME),
        os.path.join(MOON_TEMPLATES_SUBFOLDER, CRESCENT_TEMPLATES_SUBFOLDER_NAME),
        os.path.join(MOON_TEMPLATES_SUBFOLDER, HALF_TEMPLATES_SUBFOLDER_NAME),
        os.path.join(MOON_TEMPLATES_SUBFOLDER, ELLIPSE_TEMPLATES_SUBFOLDER_NAME),
        os.path.join(MOON_TEMPLATES_SUBFOLDER, FULL_TEMPLATES_SUBFOLDER_NAME),
        SUN_TEMPLATES_SUBFOLDER,
    ]
    templates = {
        "Earth-Crescent": [],
        "Earth-Half": [],
        "Earth-Ellipse": [],
        "Earth-Full": [],
        "Moon-Crescent": [],
        "Moon-Half": [],
        "Moon-Ellipse": [],
        "Moon-Full": [],
        "Sun": [],
    }
    df = pd.read_csv(METADATA_CSV)

    for index, row in df.iterrows():
        name = str(row["Name"])
        print(name)
        x = int(row["X"])
        y = int(row["Y"])
        s = int(row["S"])
        if allFolders[0] in name:
            templates["Earth-Crescent"].append(name)
        if allFolders[1] in name:
            templates["Earth-Half"].append(name)
        if allFolders[2] in name:
            templates["Earth-Ellipse"].append(name)
        if allFolders[3] in name:
            templates["Earth-Full"].append(name)
        if allFolders[4] in name:
            templates["Moon-Crescent"].append(name)
        if allFolders[5] in name:
            templates["Moon-Half"].append(name)
        if allFolders[6] in name:
            templates["Moon-Ellipse"].append(name)
        if allFolders[7] in name:
            templates["Moon-Full"].append(name)
        if allFolders[8] in name:
            templates["Sun"].append(name)

    # Just Earth
    earths = (
        templates["Earth-Crescent"] + templates["Earth-Half"] + templates["Earth-Ellipse"] + templates["Earth-Full"]
    )
    moons = templates["Moon-Crescent"] + templates["Moon-Half"] + templates["Moon-Ellipse"] + templates["Moon-Full"]
    suns = templates["Sun"]
    if bodyType == "sun":
        singular(suns, df, bodyType="Sun", radiuses=[25])
    elif bodyType == "moon":
        singular(moons, df, bodyType="Moon", radiuses=[5, 200])
    elif bodyType == "earth":
        singular(earths, df, bodyType="Earth", radiuses=[30, 60])
    else:
        print(f"Illegal body type: {bodyType}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-b", "--body", help="Which body (earth,moon,sun)")
    ap.add_argument("-c", "--conf", help="Which configuration to generate (single, three)")
    args = vars(ap.parse_args())

    if str(args["conf"]) == "single":
        generateSingleImages(args["body"])
    elif str(args["conf"]) == "three":
        three(
            earthPath=os.path.join(
                "D:\\OpNav\\data\\DetectorDatasetGenerator\\Earth\\Crescent\\crescentearthmoon.jpg"),
            moonPath=os.path.join(
                "D:\\OpNav\\data\\DetectorDatasetGenerator\\Moon\\Crescent\\steve-donna-o-meara-waxing-crescent-moon-showing-mare-crisium-large-dark-sea-on-the-right_a-G-6110078-4990703.jpg"
            ),
            sunPath=os.path.join("D:\\OpNav\\data\\DetectorDatasetGenerator\\Sun\\36497504031_24c6270d62_b_v2.png"),
        )
