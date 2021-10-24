from OpticalNavigation.core.find_with_contours import *
from OpticalNavigation.core.const import ImageDetectionCircles
import unittest
import glob
import os

class Detections(unittest.TestCase):
    
    # Very similar to function definition of the same name in opnav.py
    def get_detections(self, frames):
        #These arrays take the form (number if frame number): [[x0,y0,z0,diameter0], [x1,y1,z1,diameter1], ...]
        earthDetectionArray = np.zeros((len(frames), 4), dtype=float)
        moonDetectionArray = np.zeros((len(frames), 4), dtype=float)
        sunDetectionArray = np.zeros((len(frames), 4), dtype=float)
        progress = 1
        for f in range(len(frames)):
            imageDetectionCircles = find(frames[f]) # Puts results in ImageDetectionCircles object which is then accessed by next lines
            earthDetectionArray[f, ...] = imageDetectionCircles.get_earth_detection()
            moonDetectionArray[f, ...] = imageDetectionCircles.get_moon_detection()
            sunDetectionArray[f, ...] = imageDetectionCircles.get_sun_detection()
            progress += 1
        return earthDetectionArray, moonDetectionArray, sunDetectionArray    
    
    
    
    def test_get_truth_detections(self):
#        with open('traj.csv') as f:
#            for line in f:
#                tokens = [float(t) for t in line.split(',')]
#                t0 = tokens[0]
#                moonPos = vec3(tokens[4], tokens[5], tokens[6])
#                sunPos = vec3(tokens[7], tokens[8], tokens[9])
#                cameraPos = vec3(tokens[1], tokens[2], tokens[3])
        filenames = []
        earthDetect = []
        moonDetect = []
        sunDetect = []

        # Get detection truth values from csv
        with open('/home/stephen/Desktop/truth_detections_case1c.csv') as f:
            for line in f:
                tokens = [t for t in line.split(',')]
                filenames.append(tokens[1])
                earthDetect.append(tokens[2])
                moonDetect.append(tokens[3])
                sunDetect.append(tokens[4])
                
        # Binarizes detections and pairs them this their corresponding filename                
        earthDetect = list(zip(filenames[1:], [0 if i == '' else 1 for i in earthDetect[1:]]))
        #print(earthDetect)

        frames = glob.glob("/home/stephen/Desktop/surrender_images/cislunar_case1c/*.jpg")
        earthExp, moonExp, sunExp = self.get_detections(frames)
        earthDetectExp = [0 if np.isnan(i[0]) else 1 for i in earthExp]
        print(earthDetectExp)
        frames = [os.path.basename(path) for path in frames]
        earthDetectExp = list(zip(frames, earthDetectExp))
        print(earthDetectExp)


if __name__ == '__main__':
    unittest.main()           
    #test_get_truth_detections()