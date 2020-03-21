import unittest
import glob
import os
import cv2
import copy
import pandas as pd
from core.const import TEST_ECLIPSEANDCRESCENTIMAGES
from core.find import findEarth, findMoon, findSun

class TestFindMethods(unittest.TestCase):

    def test_earth(self):
        # Read CSV for detection ground truths
        ans = pd.read_csv(TEST_ECLIPSEANDCRESCENTIMAGES + "\\circles\\circles.csv")
        print(ans)
        loc = TEST_ECLIPSEANDCRESCENTIMAGES + "\\images\\*jpg"
        files = glob.glob(loc)
        if len(files) == 0:
            if not os.path.isdir(loc):  
                self.fail('\"{}\" is not a valid find dataset directory'.format(loc))
            else:
                # TODO: When one camera doesn't output images
                self.fail('No images found in dataset directory \"{}\"'.format(loc))
        for i, file in enumerate(files):
            print(file)
            true_img = cv2.imread(file)
            img = copy.copy(true_img)
            earthCircles = findEarth(img)
            img = copy.copy(true_img)
            moonCircles = findMoon(img)
            img = copy.copy(true_img)
            sunCircles = findSun(img)
        self.assertEqual('foo'.upper(), 'FOO')

    def test_isupper(self):
        self.assertTrue('FOO'.isupper())
        self.assertFalse('Foo'.isupper())

    def test_split(self):
        s = 'hello world'
        self.assertEqual(s.split(), ['hello', 'world'])
        # check that s.split fails when the separator is not a string
        with self.assertRaises(TypeError):
            s.split(2)

if __name__ == '__main__':
    unittest.main()