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
        earthDetect = dict(list(zip(filenames[1:], [0 if i == '' else 1 for i in earthDetect[1:]])))
        moonDetect = dict(list(zip(filenames[1:], [0 if i == '' else 1 for i in moonDetect[1:]])))
        sunDetect = dict(list(zip(filenames[1:], [0 if i == '' else 1 for i in sunDetect[1:]])))
        
        frames = glob.glob("/home/stephen/Desktop/surrender_images/cislunar_case1c/*.jpg")
        
        earthExp, moonExp, sunExp = self.get_detections(frames)
        frames = [os.path.basename(path) for path in frames]

        earthDetectExp = [0 if np.isnan(i[0]) else 1 for i in earthExp]
        earthDetectExp = dict(list(zip(frames, earthDetectExp)))
        
        moonDetectExp = [0 if np.isnan(i[0]) else 1 for i in moonExp]
        moonDetectExp = dict(list(zip(frames, moonDetectExp)))

        sunDetectExp = [0 if np.isnan(i[0]) else 1 for i in sunExp]
        sunDetectExp = dict(list(zip(frames, sunDetectExp)))


        for i in earthDetect:
            if earthDetect[i] == earthDetectExp[i]:
                pass
            else:
                print(i)

        for i in moonDetect:
            if moonDetect[i] == moonDetectExp[i]:
                pass
            else:
                print(i)

        for i in sunDetect:
            if sunDetect[i] == sunDetectExp[i]:
                pass
            else:
                print(i)                

if __name__ == '__main__':
    unittest.main()           
    #test_get_truth_detections()