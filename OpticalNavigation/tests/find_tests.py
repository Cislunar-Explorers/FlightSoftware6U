from unittest import TestCase
import glob
import os
import cv2
import copy
import math
import pandas as pd
from tests.const import TEST_ECLIPSEANDCRESCENTIMAGES, TEST_FIND_DATASET_IMAGE_DIR, TEST_FIND_DATASET_CIRCLES_DIR
from core.find import findEarth, findMoon, findSun

class NaNError(Exception):
    def __str__(self):
        return 'NaN entry in dataset'

class NullEntry(Exception):
    def __str__(self):
        return 'If one ground truth component of [c] contains \'-\', then all components must contain it as well'

class FalsePositiveDetection(Exception):
    def __str__(self):
        return 'False positive detection'

class TrueNegativeDetection(Exception):
    def __str__(self):
        return 'True negative detection'

class TestFindOnEclipseAndCrescentImagesDataset(TestCase):

    def __init__(self, *args, **kwargs):
        super(TestFindOnEclipseAndCrescentImagesDataset, self).__init__(*args, **kwargs)
        self.EARTH_CENTER_ERROR = 20
        self.EARTH_RADIUS_ERROR = 20 # Earth should be relatively easy to detect due to its blue color
        self.SUN_CENTER_ERROR = 5
        self.SUN_RADIUS_ERROR = 5 # Sun doesn't change much, which should mean better detections
        self.MOON_CENTER_ERROR = 20
        self.MOON_RADIUS_ERROR = 30 # Out of all bodies, Moon changes in appearance the most

    def checkErrors(self, cHat, c):
        """
        Parses ground truth data and converts to float
        Caculates distance between the two centers, and difference between the radii
        [cHat]: as list of (xHat, yHat, rHat) detected circles
        [c]: (x, y, r) of ground truth circles
        Returns:
        [centerDistance]: L2 distance between the two centers
        [radiusDifference]: Difference between the two radii
        """
        x, y, r = c[0], c[1], c[2]
        # Check if any circles are detected when they shouldn't be (false positive)
        if x == '-' or y =='-' or r == '-':
            if not(x == '-' or y =='-' or r == '-'):
                raise NullEntry()
            elif cHat is not None and len(cHat) >= 1:
                raise FalsePositiveDetection()
        elif math.isnan(float(c[0])) or math.isnan(float(c[1])) or math.isnan(float(c[2])):
            raise NaNError()
        # Check if circles weren't detected when they should have been (true negative)
        elif cHat is None:
            raise TrueNegativeDetection()
        

    def calculateErrors(self, cHat, c):
        """
        Parses ground truth data and converts to float
        Caculates distance between the two centers, and difference between the radii
        [cHat]: (xHat, yHat, rHat) of detected circles
        [c]: (x, y, r) of ground truth circles
        Throws:
        Exception if NaN is detected anywhere 
        Exception if one ground truth component of [c] contain '-', then all components must contain it as well 
        Exception for false positive detections
        Exception for true negative detections
        """
        xHat, yHat, rHat, x, y, r = float(cHat[0]), float(cHat[1]), float(cHat[2]), float(c[0]), float(c[1]), float(c[2])
        centerDistance = math.sqrt((xHat - x)**2 + (yHat - y)**2)
        radiusDistance = math.fabs(rHat - r)
        return centerDistance, radiusDistance
    
    def test_earth(self):
        # Read CSV for detection ground truths
        circles_df = pd.read_csv(TEST_ECLIPSEANDCRESCENTIMAGES + TEST_FIND_DATASET_CIRCLES_DIR)
        loc = TEST_ECLIPSEANDCRESCENTIMAGES + TEST_FIND_DATASET_IMAGE_DIR
        types = (loc + '\\*.jpg', loc + '\\*.png', loc + '\\*.jpeg')
        files = []
        for extension in types:
            files.extend(glob.glob(extension))
        if len(files) == 0:
            if not os.path.isdir(loc):  
                self.fail('\"{}\" is not a valid find dataset directory'.format(loc))
            else:
                # TODO: When one camera doesn't output images
                self.fail('No images found in dataset directory \"{}\"'.format(loc))
        for i, file in enumerate(files):
            print('-----------------------')
            print('Target: {}'.format(file))
            true_img = cv2.imread(file)
            img = copy.copy(true_img)
            earthCircles = findEarth(img)
            row = circles_df.loc[circles_df['Image'] == os.path.basename(file)]
            trueEarthData = (row['EarthX'].values[0], row['EarthY'].values[0], row['EarthR'].values[0])

            try:
                self.checkErrors(earthCircles, trueEarthData)
            except Exception as e:
                self.fail('Could not calculate difference in center/radius for {} in {} --> {}'.format('Earth','file',str(e)))
            
            # No Exceptions were raised        
            earthCenterError, earthRadiusError = self.calculateErrors(earthCircles[0][0], trueEarthData)
            print('Earth Result: center error: {}, radius error: {}'.format(earthCenterError, earthRadiusError))
            self.assertLessEqual(earthCenterError, self.EARTH_CENTER_ERROR)
            self.assertLessEqual(earthRadiusError, self.EARTH_RADIUS_ERROR)

    def test_sun(self):
        # Read CSV for detection ground truths
        circles_df = pd.read_csv(TEST_ECLIPSEANDCRESCENTIMAGES + TEST_FIND_DATASET_CIRCLES_DIR)
        loc = TEST_ECLIPSEANDCRESCENTIMAGES + TEST_FIND_DATASET_IMAGE_DIR
        types = (loc + '\\*.jpg', loc + '\\*.png', loc + '\\*.jpeg')
        files = []
        for extension in types:
            files.extend(glob.glob(extension))
        if len(files) == 0:
            if not os.path.isdir(loc):  
                self.fail('\"{}\" is not a valid find dataset directory'.format(loc))
            else:
                # TODO: When one camera doesn't output images
                self.fail('No images found in dataset directory \"{}\"'.format(loc))
        for i, file in enumerate(files):
            print('-----------------------')
            print('Target: {}'.format(file))
            true_img = cv2.imread(file)
            img = copy.copy(true_img)
            sunCircles = findSun(img)
            row = circles_df.loc[circles_df['Image'] == os.path.basename(file)]
            trueSunData = (row['SunX'].values[0], row['SunY'].values[0], row['SunR'].values[0])

            try:
                self.checkErrors(sunCircles, trueSunData)
            except Exception as e:
                self.fail('Could not calculate difference in center/radius for {} in {} --> {}'.format('Sun','file',str(e)))
            
            # No Exceptions were raised        
            sunCenterError, sunRadiusError = self.calculateErrors(sunCircles[0][0], trueSunData)
            print('Sun Result: center error: {}, radius error: {}'.format(sunCenterError, sunRadiusError))
            self.assertLessEqual(sunCenterError, self.SUN_CENTER_ERROR)
            self.assertLessEqual(sunRadiusError, self.SUN_RADIUS_ERROR)

    def test_moon(self):
        # Read CSV for detection ground truths
        circles_df = pd.read_csv(TEST_ECLIPSEANDCRESCENTIMAGES + TEST_FIND_DATASET_CIRCLES_DIR)
        loc = TEST_ECLIPSEANDCRESCENTIMAGES + TEST_FIND_DATASET_IMAGE_DIR
        types = (loc + '\\*.jpg', loc + '\\*.png', loc + '\\*.jpeg')
        files = []
        for extension in types:
            files.extend(glob.glob(extension))
        if len(files) == 0:
            if not os.path.isdir(loc):  
                self.fail('\"{}\" is not a valid find dataset directory'.format(loc))
            else:
                # TODO: When one camera doesn't output images
                self.fail('No images found in dataset directory \"{}\"'.format(loc))
        for i, file in enumerate(files):
            print('-----------------------')
            print('Target: {}'.format(file))
            true_img = cv2.imread(file)
            img = copy.copy(true_img)
            moonCircles = findMoon(img)
            row = circles_df.loc[circles_df['Image'] == os.path.basename(file)]
            trueMoonData = (row['MoonX'].values[0], row['MoonY'].values[0], row['MoonR'].values[0])

            try:
                self.checkErrors(moonCircles, trueMoonData)
            except Exception as e:
                self.fail('Could not calculate difference in center/radius for {} in {} --> {}'.format('Sun','file',str(e)))
            
            # No Exceptions were raised        
            moonCenterError, moonRadiusError = self.calculateErrors(moonCircles[0][0], trueMoonData)
            print('Moon Result: center error: {}, radius error: {}'.format(moonCenterError, moonRadiusError))
            self.assertLessEqual(moonCenterError, self.MOON_CENTER_ERROR)
            self.assertLessEqual(moonRadiusError, self.MOON_RADIUS_ERROR)


if __name__ == '__main__':
    unittest.main()