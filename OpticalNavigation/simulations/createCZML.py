import os
import pandas as pd
import numpy as np
import json
from tqdm import tqdm

from sim import isOrthogonal, attitudeMatrix
from attitudeSim import getCameraVectors, rotateVector


def output_CZML(trajDfpath, attDfPath, original_csv_timescale_in_seconds, startDate, endDate):
    """
    [original_csv_timescale_in_seconds] is used to label each row with the right time (in seconds), where the
    first row is equal to 0 seconds.
    """
    outDict = [
        {
            "id":"document",
            "name": "CZML Stream",
            "version": "1.0",
            "clock": 
            {
                "intervals": startDate + "/" + endDate,
                "currentTime": startDate,
                "multiplier": 1
            }
        },
        {
            "id": "CislunarExplorers",
            "availability": startDate + "/" + endDate,
            "position": 
            {
                "interpolationAlgorithm": "LAGRANGE",
                "interpolationDegree": 5,
                "referenceFrame": "INERTIAL",
                "epoch": startDate,
                "cartesian": []
            }
        },
        {
            "id": "camera 1 view 1",
            "availability": startDate + "/" + endDate,
            "position": 
            {
                "interpolationAlgorithm": "LAGRANGE",
                "interpolationDegree": 5,
                "referenceFrame": "INERTIAL",
                "epoch": startDate,
                "cartesian": []
            }
        },
        {
            "id": "camera 1 view 2",
            "availability": startDate + "/" + endDate,
            "position": 
            {
                "interpolationAlgorithm": "LAGRANGE",
                "interpolationDegree": 5,
                "referenceFrame": "INERTIAL",
                "epoch": startDate,
                "cartesian": []
            }
        },
        {
            "id": "camera 1 view 3",
            "availability": startDate + "/" + endDate,
            "position": 
            {
                "interpolationAlgorithm": "LAGRANGE",
                "interpolationDegree": 5,
                "referenceFrame": "INERTIAL",
                "epoch": startDate,
                "cartesian": []
            }
        },
        {
            "id": "camera 1 view 4",
            "availability": startDate + "/" + endDate,
            "position": 
            {
                "interpolationAlgorithm": "LAGRANGE",
                "interpolationDegree": 5,
                "referenceFrame": "INERTIAL",
                "epoch": startDate,
                "cartesian": []
            }
        },
        {
            "id": "camera 1 view 5",
            "availability": startDate + "/" + endDate,
            "position": 
            {
                "interpolationAlgorithm": "LAGRANGE",
                "interpolationDegree": 5,
                "referenceFrame": "INERTIAL",
                "epoch": startDate,
                "cartesian": []
            }
        },
        {
            "id": "camera 1 view 6",
            "availability": startDate + "/" + endDate,
            "position": 
            {
                "interpolationAlgorithm": "LAGRANGE",
                "interpolationDegree": 5,
                "referenceFrame": "INERTIAL",
                "epoch": startDate,
                "cartesian": []
            }
        },
        {
            "id": "camera 1 view 7",
            "availability": startDate + "/" + endDate,
            "position": 
            {
                "interpolationAlgorithm": "LAGRANGE",
                "interpolationDegree": 5,
                "referenceFrame": "INERTIAL",
                "epoch": startDate,
                "cartesian": []
            }
        },
        {
            "id": "camera 1 view 8",
            "availability": startDate + "/" + endDate,
            "position": 
            {
                "interpolationAlgorithm": "LAGRANGE",
                "interpolationDegree": 5,
                "referenceFrame": "INERTIAL",
                "epoch": startDate,
                "cartesian": []
            }
        },
        {
            "id": "camera 2 view 1",
            "availability": startDate + "/" + endDate,
            "position": 
            {
                "interpolationAlgorithm": "LAGRANGE",
                "interpolationDegree": 5,
                "referenceFrame": "INERTIAL",
                "epoch": startDate,
                "cartesian": []
            }
        },
        {
            "id": "camera 2 view 2",
            "availability": startDate + "/" + endDate,
            "position": 
            {
                "interpolationAlgorithm": "LAGRANGE",
                "interpolationDegree": 5,
                "referenceFrame": "INERTIAL",
                "epoch": startDate,
                "cartesian": []
            }
        },
        {
            "id": "camera 2 view 3",
            "availability": startDate + "/" + endDate,
            "position": 
            {
                "interpolationAlgorithm": "LAGRANGE",
                "interpolationDegree": 5,
                "referenceFrame": "INERTIAL",
                "epoch": startDate,
                "cartesian": []
            }
        },
        {
            "id": "camera 2 view 4",
            "availability": startDate + "/" + endDate,
            "position": 
            {
                "interpolationAlgorithm": "LAGRANGE",
                "interpolationDegree": 5,
                "referenceFrame": "INERTIAL",
                "epoch": startDate,
                "cartesian": []
            }
        },
        {
            "id": "camera 2 view 5",
            "availability": startDate + "/" + endDate,
            "position": 
            {
                "interpolationAlgorithm": "LAGRANGE",
                "interpolationDegree": 5,
                "referenceFrame": "INERTIAL",
                "epoch": startDate,
                "cartesian": []
            }
        },
        {
            "id": "camera 2 view 6",
            "availability": startDate + "/" + endDate,
            "position": 
            {
                "interpolationAlgorithm": "LAGRANGE",
                "interpolationDegree": 5,
                "referenceFrame": "INERTIAL",
                "epoch": startDate,
                "cartesian": []
            }
        },
        {
            "id": "camera 2 view 7",
            "availability": startDate + "/" + endDate,
            "position": 
            {
                "interpolationAlgorithm": "LAGRANGE",
                "interpolationDegree": 5,
                "referenceFrame": "INERTIAL",
                "epoch": startDate,
                "cartesian": []
            }
        },
        {
            "id": "camera 2 view 8",
            "availability": startDate + "/" + endDate,
            "position": 
            {
                "interpolationAlgorithm": "LAGRANGE",
                "interpolationDegree": 5,
                "referenceFrame": "INERTIAL",
                "epoch": startDate,
                "cartesian": []
            }
        },
        {
            "id": "camera 3 view 1",
            "availability": startDate + "/" + endDate,
            "position": 
            {
                "interpolationAlgorithm": "LAGRANGE",
                "interpolationDegree": 5,
                "referenceFrame": "INERTIAL",
                "epoch": startDate,
                "cartesian": []
            }
        },
        {
            "id": "camera 3 view 2",
            "availability": startDate + "/" + endDate,
            "position": 
            {
                "interpolationAlgorithm": "LAGRANGE",
                "interpolationDegree": 5,
                "referenceFrame": "INERTIAL",
                "epoch": startDate,
                "cartesian": []
            }
        },
        {
            "id": "camera 3 view 3",
            "availability": startDate + "/" + endDate,
            "position": 
            {
                "interpolationAlgorithm": "LAGRANGE",
                "interpolationDegree": 5,
                "referenceFrame": "INERTIAL",
                "epoch": startDate,
                "cartesian": []
            }
        },
        {
            "id": "camera 3 view 4",
            "availability": startDate + "/" + endDate,
            "position": 
            {
                "interpolationAlgorithm": "LAGRANGE",
                "interpolationDegree": 5,
                "referenceFrame": "INERTIAL",
                "epoch": startDate,
                "cartesian": []
            }
        },
        {
            "id": "camera 3 view 5",
            "availability": startDate + "/" + endDate,
            "position": 
            {
                "interpolationAlgorithm": "LAGRANGE",
                "interpolationDegree": 5,
                "referenceFrame": "INERTIAL",
                "epoch": startDate,
                "cartesian": []
            }
        },
        {
            "id": "camera 3 view 6",
            "availability": startDate + "/" + endDate,
            "position": 
            {
                "interpolationAlgorithm": "LAGRANGE",
                "interpolationDegree": 5,
                "referenceFrame": "INERTIAL",
                "epoch": startDate,
                "cartesian": []
            }
        },
        {
            "id": "camera 3 view 7",
            "availability": startDate + "/" + endDate,
            "position": 
            {
                "interpolationAlgorithm": "LAGRANGE",
                "interpolationDegree": 5,
                "referenceFrame": "INERTIAL",
                "epoch": startDate,
                "cartesian": []
            }
        },
        {
            "id": "camera 3 view 8",
            "availability": startDate + "/" + endDate,
            "position": 
            {
                "interpolationAlgorithm": "LAGRANGE",
                "interpolationDegree": 5,
                "referenceFrame": "INERTIAL",
                "epoch": startDate,
                "cartesian": []
            }
        },
        {
            "id": "camera up",
            "availability": startDate + "/" + endDate,
            "position": 
            {
                "interpolationAlgorithm": "LAGRANGE",
                "interpolationDegree": 5,
                "referenceFrame": "INERTIAL",
                "epoch": startDate,
                "cartesian": []
            }
        }
        
    ]

    trajDf = pd.read_csv(trajDfpath)
    attDf = pd.read_csv(attDfPath)
    timestep = original_csv_timescale_in_seconds # Sample rate of original csv in seconds
    time = list(np.arange(0,len(trajDf.index)*original_csv_timescale_in_seconds, timestep))
    x = np.array(trajDf['x'].tolist(), dtype=np.float) * 1000
    y = np.array(trajDf['y'].tolist(), dtype=np.float) * 1000
    z = np.array(trajDf['z'].tolist(), dtype=np.float) * 1000
    # vx = np.array(trajDf['vx'].tolist(), dtype=np.float) * 1000
    # vy = np.array(trajDf['vy'].tolist(), dtype=np.float) * 1000
    # vz = np.array(trajDf['vz'].tolist(), dtype=np.float) * 1000
    q1 = np.array(attDf['q1'].tolist(), dtype=np.float)
    q2 = np.array(attDf['q2'].tolist(), dtype=np.float)
    q3 = np.array(attDf['q3'].tolist(), dtype=np.float)
    q4 = np.array(attDf['q4'].tolist(), dtype=np.float)

    for i in tqdm(range(len(time)), desc="Writing to CZML"):
        # vector = np.array([vx[i], vy[i], vz[i]], dtype=np.float)
        # vector = vector / np.linalg.norm(vector)

        position_string = [int(time[i]),float(x[i]),float(y[i]),float(z[i])]

        # Extract quaternions
        q = np.array([float(q1[i]), float(q2[i]), float(q3[i]), float(q4[i])])
        assert abs(np.linalg.norm(q) - 1) < 1e-3
        Aq = attitudeMatrix(q)
        # X axis of spacecraft
        local_vector = np.array([1, 0, 0, 0]) # last 0 is padding
        X_A = np.dot(Aq,local_vector[:3].T.reshape(3,1))
        # Y axis of spacecraft
        local_vector = np.array([0, 1, 0, 0]) # last 0 is padding
        Y_A = np.dot(Aq,local_vector[:3].T.reshape(3,1))

        # Z axis of spacecraft
        local_vector = np.array([0, 0, 1, 0]) # last 0 is padding
        Z_A = np.dot(Aq,local_vector[:3].T.reshape(3,1))
        # get camera first viewpoint
        camera1vector, camera2vector, camera3vector = getCameraVectors(Y_A.flatten(), Z_A.flatten())
        cam1string = [int(time[i]),float(camera1vector[0]),float(camera1vector[1]),float(camera1vector[2])]
        cam2string = [int(time[i]),float(camera2vector[0]),float(camera2vector[1]),float(camera2vector[2])]
        cam3string = [int(time[i]),float(camera3vector[0]),float(camera3vector[1]),float(camera3vector[2])]
        
        for view_index, view in enumerate(np.arange(0, 360, 360/8)):
            camera1Index = view_index+2
            camera2Index = 8+view_index+2
            camera3Index = 8+8+view_index+2
            camera1vector = rotateVector(camera1vector, X_A.flatten(), view)
            camera2vector = rotateVector(camera2vector, X_A.flatten(), view)
            camera3vector = rotateVector(camera3vector, X_A.flatten(), view)
            outDict[camera1Index]['position']['cartesian'].extend(cam1string)
            outDict[camera2Index]['position']['cartesian'].extend(cam2string)
            outDict[camera3Index]['position']['cartesian'].extend(cam3string)
            outDict[-1]['position']['cartesian'].extend(list(Z_A.flatten()))
    
        outDict[1]['position']['cartesian'].extend(position_string)
        # outDict[2]['velocity']['cartesian'].extend(velocity_string)
    
    json_object = json.dumps(outDict, indent = 4) 
    print(json_object)  


if __name__ == "__main__":
    # The sampled trajectory CSV
    INITIAL_TRAJ_PATH = os.path.join('D:', 'OpNav', 'data', 'CislunarFullTraj_60secs', 'trajectory', '1min_stk_active_sampled_traj.csv')
    # Attitude CSV based on maneuver data (getManeuver.py)
    INITIAL_ATT_PATH = os.path.join('D:', 'OpNav', 'data', 'CislunarFullTraj_60secs', 'attitude', 'discrete_att_with_maneuver_nutations.csv')

    startDate = "2018-10-09T16:15:34.0000Z"
    endDate = "2019-08-12T02:15:34.0000Z"

    output_CZML(INITIAL_TRAJ_PATH, INITIAL_ATT_PATH, 60, startDate, endDate)


    # endDate = ""

    # for index, row in moonDf.iterrows():
    #     time.append(index)
    #     x.append(trajDf['x'][index])