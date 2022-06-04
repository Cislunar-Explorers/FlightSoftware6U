import numpy as np
import glob
import cv2
import copy
import os
from core.find_algos.find import findEarth, findMoon, findSun
from core.const import (
    NoImagesInCameraAcquisitionDirectory,
    InvalidBodyNameForLoadProperties,
)


def loadProperties(img, name, cam, i, properties):
    """
    Populates properties with detected metadata
    [img]: Original image
    [name]: string id of body of interest
    [cam]: camera id
    [i]: image id / time offset
    [properties]: dict for detection metadata
    Returns:
    Populates properties dict if detection successful
    Throws:
    Exception is incorrect body name provided
    """
    if name == "earth":
        circles = findEarth(img)
    elif name == "moon":
        circles = findMoon(img)
    elif name == "sun":
        circles = findSun(img)
    else:
        raise InvalidBodyNameForLoadProperties(name)
    if circles is not None and len(circles) != 0:
        print("{}: Sucess in Camera {} Image {}".format(name, cam + 1, i + 1))
        bestCircle = circles[0][0]
        properties["center"].append(np.array([bestCircle[0], bestCircle[1]]))
        properties["radius"].append(float(bestCircle[2]))
        properties["index"].append(i)
        properties["flag"] = cam


def computeMeasurement(
    earthProperties, moonProperties, sunProperties, omega, dt, cameraParameters
):
    # Time Difference between snaps of bodies
    dtEM = np.abs(earthProperties["index"][0] - moonProperties["index"][0]) * dt
    dtES = np.abs(earthProperties["index"][0] - sunProperties["index"][0]) * dt
    dtMS = np.abs(moonProperties["index"][0] - sunProperties["index"][0]) * dt
    # Angle Between Bodies (Convert to Degrees)
    angEM = np.mod(dtEM * omega[2], 2 * np.pi) * 180 / np.pi
    angES = np.mod(dtES * omega[2], 2 * np.pi) * 180 / np.pi
    angMS = np.mod(dtMS * omega[2], 2 * np.pi) * 180 / np.pi
    one_zero = np.array([1, 0])
    zero_one = np.array([0, 1])
    # Additional Horizontal Pixels Due to Time Offset
    horiz_pixelsEM = angEM / cameraParameters.hFov * cameraParameters.hPix * one_zero
    horiz_pixelsES = angES / cameraParameters.hFov * cameraParameters.hPix * one_zero
    horiz_pixelsMS = angMS / cameraParameters.hFov * cameraParameters.hPix * one_zero
    # Additional Vertical Pixels Due to Camera Position offset
    vert_pixels12 = (
        cameraParameters.dcam12 / cameraParameters.vFov * cameraParameters.vPix
    )
    vert_pixels13 = (
        cameraParameters.dcam13 / cameraParameters.vFov * cameraParameters.vPix
    )
    vert_pixels23 = (
        cameraParameters.dcam23 / cameraParameters.vFov * cameraParameters.vPix
    )
    vertPix = np.array(
        [
            [0, vert_pixels12, vert_pixels13],
            [vert_pixels12, 0, vert_pixels23],
            [vert_pixels13, vert_pixels23, 0],
        ]
    )
    # Measurements
    z1 = np.linalg.norm(
        np.abs(earthProperties["center"][0] - moonProperties["center"][0])
        + horiz_pixelsEM
        + vertPix[earthProperties["flag"]][moonProperties["flag"]] * zero_one
    )
    z2 = np.linalg.norm(
        np.abs(earthProperties["center"][0] - sunProperties["center"][0])
        + horiz_pixelsES
        + vertPix[earthProperties["flag"]][sunProperties["flag"]] * zero_one
    )
    z3 = np.linalg.norm(
        np.abs(sunProperties["center"][0] - moonProperties["center"][0])
        + horiz_pixelsMS
        + vertPix[moonProperties["flag"]][sunProperties["flag"]] * zero_one
    )
    z4 = 2 * earthProperties["radius"][0]
    z5 = 2 * moonProperties["radius"][0]
    z6 = 2 * sunProperties["radius"][0]

    return np.array([z1, z2, z3, z4, z5, z6])


def cameraMeasurements(omega, dt, dir, cameraParameters):
    """
    Generates measurement vector from images
    [omega]: 3 x 1 angular velocity vector (rad/s)
    [dt]: time difference between consecutive photos (s)
    [dir]: Directory of acquired images. Should have subfolders Camera1/, Camera2/, Camera3/
    [cameraParameters]: camera settings used to take the photos
    Returns:
    [h] 6 x 1 matrix of measured values [z1 ... z6]
        z1,z2,z3 = angular distance between EM, ES, and MS (pixels)
        z4,z5,z6 = apparent diameter of E,M,S (pixels)
    [None] if one body could not be found
    Throws:
    Exception if camera acquisition direction doesn't exist or doesn't contain images
    """
    earthProperties = {"center": [], "radius": [], "index": [], "flag": None}
    sunProperties = {"center": [], "radius": [], "index": [], "flag": None}
    moonProperties = {"center": [], "radius": [], "index": [], "flag": None}

    cameraLocations = []
    cameraLocations.append(os.path.join(dir, "Camera1"))
    cameraLocations.append(os.path.join(dir, "Camera2"))
    cameraLocations.append(os.path.join(dir, "Camera3"))

    # TODO: Verify that detected radii are consistent across all images
    for cam, camLoc in enumerate(cameraLocations):
        types = (
            os.path.join(camLoc, "*.jpg"),
            os.path.join(camLoc, "*.png"),
            os.path.join(camLoc, "*.jpeg"),
        )
        files = []
        for extension in types:
            files.extend(glob.glob(extension))
        if len(files) == 0:
            if not os.path.isdir(camLoc):
                raise InvalidBodyNameForLoadProperties(camLoc)
            else:
                # TODO: When one camera doesn't output images
                raise NoImagesInCameraAcquisitionDirectory(camLoc)
        for i, file in enumerate(files):
            true_img = cv2.imread(file)
            try:
                # Find Earth
                if len(earthProperties["index"]) == 0:
                    loadProperties(
                        copy.copy(true_img), "earth", cam, i, earthProperties
                    )
                # Find Moon
                if len(moonProperties["index"]) == 0:
                    loadProperties(copy.copy(true_img), "moon", cam, i, moonProperties)
                # Find Sun
                if len(sunProperties["index"]) == 0:
                    loadProperties(copy.copy(true_img), "sun", cam, i, sunProperties)
            except InvalidBodyNameForLoadProperties as e:
                raise e

    print(earthProperties, moonProperties, sunProperties)
    # TODO: A body was not found
    atleastOneBodyNotFound = (
        len(earthProperties["index"]) == 0
        or len(moonProperties["index"]) == 0
        or len(sunProperties["index"]) == 0
    )
    if len(earthProperties["index"]) == 0:
        print("No Earth found")
    if len(moonProperties["index"]) == 0:
        print("No Moon found")
    if len(sunProperties["index"]) == 0:
        print("No Sun found")
    if atleastOneBodyNotFound:
        return None

    return computeMeasurement(
        earthProperties, moonProperties, sunProperties, omega, dt, cameraParameters
    )
