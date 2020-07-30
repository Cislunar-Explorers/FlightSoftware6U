import argparse
import numpy as np
import os
import pandas as pd

"""
Extracts 6-D measurement vectors from circlesCombined.csv (see detectCesiumObject.py).
"""

def calculateMeasurement(earth, moon, sun):
    """
    Calculates measurement based on detections
    [earth,moon,sun] dictionaries with camera, viewpoint, center and radius information
    Returns 6-D vector measurement
    Layout of view point grid:
            0   45  90  135 180 225 270 315
    +60 c3  
    0   c1
    -60 c2
    """
    # Cesium image size
    W = 640
    H = 480
    VIEWS = 8
    CAM_TO_V_OFFSET = {3:0, 1:H, 2:(H+H)}
    # Calculate earth position
    earthHOffset = (earth['view']%VIEWS) * W
    earthX = (earth['pos'][0]) + earthHOffset
    earthVOffset = (CAM_TO_V_OFFSET[earth['cam']])
    earthY = earth['pos'][1] + earthVOffset
    earthD = earth['pos'][2] * 2

    # Calculate moon position
    moonHOffset = (moon['view']%VIEWS) * W
    moonX = moon['pos'][0] + moonHOffset
    moonVOffset = (CAM_TO_V_OFFSET[moon['cam']])
    moonY = moon['pos'][1] + moonVOffset
    moonD = moon['pos'][2] * 2

    # Calculate sun position
    sunHOffset = (sun['view']%VIEWS) * W
    sunX = sun['pos'][0] + sunHOffset
    sunVOffset = (CAM_TO_V_OFFSET[sun['cam']])
    sunY = sun['pos'][1] + sunVOffset
    sunD = sun['pos'][2] * 2

    EM = np.linalg.norm(np.abs(np.array([earthX-moonX, earthY-moonY])))
    ES = np.linalg.norm(np.abs(np.array([earthX-sunX, earthY-sunY])))
    MS = np.linalg.norm(np.abs(np.array([sunX-moonX, sunY-moonY])))

    return [EM, MS, ES, earthD, moonD, sunD]



def extractMeasurements(circlesDf, measurementDir):
    """
    Calculates 6D measurement vector based on detections. If there are multiple detections,
    then any combination should produce reasonably similar measurements.
    [circlesDf]: circlesCombined.csv file produced by detectCesiumObjects.csv
    [measurementDir]: save directory for measurements csv
    Saves measurements csv in the directory
    """
    circleDict = circlesDf.to_dict('list')
    currentIteration = 0
    index = 0
    earths = {'cam':[], 'view':[], 'pos':[]}
    moons = {'cam':[], 'view':[], 'pos':[]}
    suns = {'cam':[], 'view':[], 'pos':[]}
    measurements = {'iteration':[], 'Z1':[], 'Z2':[], 'Z3':[], 'Z4':[], 'Z5':[], 'Z6':[]}

    while index < len(circleDict['Iteration']):
        if(circleDict['Iteration'][index] != currentIteration):
            print(currentIteration)
            if not(len(earths['cam']) >= 1 and len(moons['cam']) >= 1 and len(suns['cam']) >= 1):
                # Something went wrong with detection as all three bodies were not detected
                break
            # Calculate measurements
            measurement = [float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), float('inf')]
            for e in range(len(earths['cam'])):
                for m in range(len(moons['cam'])):
                    for s in range(len(suns['cam'])):

                        m = calculateMeasurement( {'cam':earths['cam'][e], 'view':earths['view'][e], 'pos':earths['pos'][e]},
                                                            {'cam':moons['cam'][m], 'view':moons['view'][m], 'pos':moons['pos'][m]},
                                                            {'cam':suns['cam'][s], 'view':suns['view'][s], 'pos':suns['pos'][s]}
                                                            )
                        # Keep most direct relative distances
                        # Average all detected radii
                        measurement[0] = min(measurement[0], m[0])
                        measurement[1] = min(measurement[1], m[1])
                        measurement[2] = min(measurement[2], m[2])
                        measurement[3] = min(measurement[3], m[3])
                        measurement[4] = min(measurement[4], m[4])
                        measurement[5] = min(measurement[5], m[5])

                        print(measurement)
            measurements['iteration'].append(currentIteration)
            measurements['Z1'].append(measurement[0])
            measurements['Z2'].append(measurement[1])
            measurements['Z3'].append(measurement[2])
            measurements['Z4'].append(measurement[3])
            measurements['Z5'].append(measurement[4])
            measurements['Z6'].append(measurement[5])

            earths = {'cam':[], 'view':[], 'pos':[]}
            moons = {'cam':[], 'view':[], 'pos':[]}
            suns = {'cam':[], 'view':[], 'pos':[]}
            # Next iteration
            currentIteration = circleDict['Iteration'][index]
        
        # Earth exists
        if circleDict['EX'][index] != '-':
            earths['cam'].append(int(circleDict['Camera'][index]))
            earths['view'].append(int(circleDict['View'][index]))
            earths['pos'].append([float(circleDict['EX'][index]),float(circleDict['EY'][index]),float(circleDict['ER'][index])])
        # Moon exists
        if circleDict['MX'][index] != '-':
            moons['cam'].append(int(circleDict['Camera'][index]))
            moons['view'].append(int(circleDict['View'][index]))
            moons['pos'].append([float(circleDict['MX'][index]),float(circleDict['MY'][index]),float(circleDict['MR'][index])])
        # Sun exists
        if circleDict['SX'][index] != '-':
            suns['cam'].append(int(circleDict['Camera'][index]))
            suns['view'].append(int(circleDict['View'][index]))
            suns['pos'].append([float(circleDict['SX'][index]),float(circleDict['SY'][index]),float(circleDict['SR'][index])])
        index += 1

    # for index, row in circlesDf.iterrows():
    df = pd.DataFrame.from_dict(measurements)
    df.to_csv(os.path.join(measurementDir, 'meas_min.csv'), index=False)
        

if __name__ == "__main__":
    """
    Run "python FilterCesiumSun.py -i=<IMAGE>" to test this module
    """
    ap = argparse.ArgumentParser()
    ap.add_argument("-c", "--circlecombined", help = "path to the circlesCombined.csv file")
    ap.add_argument("-m", "--measurements", help = "path to the measurements folder")
    args = vars(ap.parse_args())

    circlesCSV = pd.read_csv(args['circlecombined'])

    extractMeasurements(circlesCSV, args['measurements'])
