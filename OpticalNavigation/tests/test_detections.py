from OpticalNavigation.core.find_with_contours import *
from OpticalNavigation.core.const import ImageDetectionCircles
import unittest
import glob
import os

# Change these two paths for your own testing
repoPath = "/home/stephen/code/FlightSoftware/"
surrenderBase = "/home/stephen/Desktop/surrender_images/"

truthValueFile = repoPath + "OpticalNavigation/tests/truth_detections_case1c.csv"
surrenderPath = surrenderBase + "cislunar_case1c/*.jpg"

class Detections(unittest.TestCase):
    """Tests the detection accuracy of a given find algorithm. This tests whether we are able to 
    detect the presence of the earth, moon, or sun in a set of images. The truth data is hand analyzed
    and the data is populated in a csv file."""

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
    
    # Test to get detection accuracy
    def test_get_truth_detections(self):
        filenames = []
        earthDetect = []
        moonDetect = []
        sunDetect = []

        # Get detection truth values from csv
        with open(truthValueFile) as f:
            for line in f:
                tokens = [t for t in line.split(',')]
                filenames.append(tokens[0]) # Second col is filename
                earthDetect.append(tokens[1]) # Third column is earth presence
                moonDetect.append(tokens[2]) # And so on...
                sunDetect.append(tokens[3])
                
        # Binarizes detections and pairs them this their corresponding filename
        # dict{key:filename, value:0/1}             
        earthDetect = dict(list(zip(filenames[1:], [0 if i == '' else 1 for i in earthDetect[1:]])))
        moonDetect = dict(list(zip(filenames[1:], [0 if i == '' else 1 for i in moonDetect[1:]])))
        sunDetect = dict(list(zip(filenames[1:], [0 if i == '' else 1 for i in sunDetect[1:]])))
        
        frames = glob.glob(surrenderPath)
        
        earthExp, moonExp, sunExp = self.get_detections(frames)
        frames = [os.path.basename(path) for path in frames]

        # Pairs filename and detection based on find algorithm
        # dict{key:filename, value:0/1}
        earthDetectExp = [0 if np.isnan(i[0]) else 1 for i in earthExp]
        earthDetectExp = dict(list(zip(frames, earthDetectExp)))
        
        moonDetectExp = [0 if np.isnan(i[0]) else 1 for i in moonExp]
        moonDetectExp = dict(list(zip(frames, moonDetectExp)))

        sunDetectExp = [0 if np.isnan(i[0]) else 1 for i in sunExp]
        sunDetectExp = dict(list(zip(frames, sunDetectExp)))

        # Accumulators for metrics
        total = len(frames)
        earthTotal = 0
        earthWrong = 0
        moonTotal = 0
        moonWrong = 0
        sunTotal = 0
        sunWrong = 0

        for i in earthDetect:
            if earthDetect[i] == earthDetectExp[i] == 1:
                earthTotal += 1
            elif earthDetect[i] != earthDetectExp[i]:
                print("Wrong Earth: ", i)
                earthWrong += 1
                earthTotal += 1

        for i in moonDetect:
            if moonDetect[i] == moonDetectExp[i] == 1:
                moonTotal += 1
            elif moonDetect[i] != moonDetectExp[i]:
                print("Wrong Moon: ", i)
                moonWrong += 1
                moonTotal += 1
            
        for i in sunDetect:
            if sunDetect[i] == sunDetectExp[i] == 1:
                sunTotal += 1
            elif sunDetect[i] != sunDetectExp[i]:
                print("Sun Wrong: ", i)
                sunWrong += 1
                sunTotal += 1

        print("Earth accuracy: ", earthTotal - earthWrong, "/", earthTotal, "=", 1 - earthWrong/earthTotal,)
        print("Moon accuracy: ", moonTotal - moonWrong, "/", moonTotal, "=", 1 - moonWrong/moonTotal)
        print("Sun accuracy: ", sunTotal - sunWrong, "/", sunTotal, "=", 1 - sunWrong/sunTotal)
        print("Total accuracy: ", total - (earthWrong + moonWrong + sunWrong), "/", total, "=", 1 - (earthWrong + moonWrong + sunWrong) / total)

        #self.assertEqual(earthWrong + moonWrong + sunWrong, 0, "Find algorithm is not 100% accurate!")
            
if __name__ == '__main__':
    unittest.main()