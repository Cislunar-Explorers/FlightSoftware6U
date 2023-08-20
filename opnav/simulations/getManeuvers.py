from datetime import datetime
import pandas as pd
import numpy as np
from tqdm import tqdm
from fsw.opnav.tests.gen_opnav_data import generateSyntheticData
import argparse
import os

monthsToNum = {
    "Jan": "01",
    "Feb": "02",
    "Mar": "03",
    "Apr": "04",
    "May": "05",
    "Jun": "06",
    "Jul": "07",
    "Aug": "08",
    "Sep": "09",
    "Oct": "10",
    "Nov": "11",
    "Dec": "12",
}


def getDateTimeFromISO(dateParts):
    """
    returns string format for datetime.
    [dateParts] = ['06', 'Mar', '2019', '01', '39', '06', 814]
    """
    assert len(dateParts) == 7
    return datetime.fromisoformat(
        dateParts[2]
        + "-"
        + monthsToNum[dateParts[1]]
        + "-"
        + dateParts[0]
        + " "
        + dateParts[3]
        + ":"
        + dateParts[4]
        + ":"
        + dateParts[5]
        + "."
        + dateParts[6]
    )


def getMissionTimeline(startEndDatesDf):
    missionStartDate = getDateTimeFromISO(
        startEndDatesDf.iloc[0, 0].split(" ")
    ).timestamp()
    missionEndDate = getDateTimeFromISO(
        startEndDatesDf.iloc[1, 0].split(" ")
    ).timestamp()
    return missionStartDate, missionEndDate


def extractCheckpoints(checkPointsDf, missionEndDate):
    """
    Generates a maneuver report for the satellite. Each maneuver
    is represented by a start and end time (in seconds).
    [missionStartDate] is a UTC timestamp (seconds)
    returns a dictionary that contains the maneuvers in time interval form
    """

    maneuvers = {"startTime": [], "endTime": []}
    for index, row in checkPointsDf.iterrows():
        startStr = row["start"].split(" ")
        endStr = row["end"].split(" ")
        startDate = getDateTimeFromISO(startStr).timestamp()
        endDate = getDateTimeFromISO(endStr).timestamp()
        if (
            startDate > missionEndDate
        ):  # Ignore some part of the mission (to fit the checkpoints in the dataset time range)
            break
        maneuvers["startTime"].append(startDate)
        maneuvers["endTime"].append(endDate)

    return maneuvers


def createDiscreteAttitudeManeuvers(
    maneuversDict, vncDf, missionStartDate, missionEndDate, missionTimeline, gyro_t
):
    """
    Creates attitude synthetic data for each maneuver using the VNC (Earth) attitude quaternions
    and sythentic attitude data propogated using nutation damping physics.

    The key drawback of this method of attitude synthesis is that the quaternions snap from
    point to point before start of each maneuver. This is due to the difficulty of simulating
    true attitude control behavior. Therefore, a continuous version of the attitude UKF
    cannot be tested along with the trajectory UKF.
    """
    sectionStartTime = missionStartDate
    sectionEndTime = None

    currentManeuver = 0
    isManeuver = False

    missionParams = {
        "q1": [0] * len(missionTimeline),
        "q2": [0] * len(missionTimeline),
        "q3": [0] * len(missionTimeline),
        "q4": [0] * len(missionTimeline),
        "wx": [0] * len(missionTimeline),
        "wy": [0] * len(missionTimeline),
        "wz": [0] * len(missionTimeline),
        "bx": [0] * len(missionTimeline),
        "by": [0] * len(missionTimeline),
        "bz": [0] * len(missionTimeline),
    }

    ############
    quat = np.array([[0.0, 0.0, 0.0, 1]]).T
    cameradt = 60  # seconds
    coldGasThrustKickTime = 0  # seconds, beginning of each propagation sequence
    coldGasKickDuration = float(10)
    omegaInit = [3, 0.1, 0.1, 0.0, 0.0, 0.0]
    biasInit = [0.0, 0.0, 0.0]
    gyroSampleCount = 1.0 / gyro_t

    gyroNoiseSigma = 1.0e-7
    gyroSigma = 1.0e-10
    # gyro_t = 1.0/gyroSampleCount
    # meas_sigma = 8.7e-4

    newData = False
    propStartTime = missionStartDate
    if currentManeuver < len(maneuversDict["startTime"]):
        propEndTime = maneuversDict["startTime"][currentManeuver]
    else:
        propEndTime = missionEndDate
    propinitQuat = quat
    ############

    for index, timestamp in enumerate(
        tqdm(missionTimeline, desc="Generating Mission Attitude")
    ):
        if currentManeuver >= 0 and currentManeuver < len(maneuversDict["startTime"]):
            isManeuver = (
                timestamp >= maneuversDict["startTime"][currentManeuver]
                and timestamp <= maneuversDict["endTime"][currentManeuver]
            )
            if timestamp > maneuversDict["endTime"][currentManeuver]:
                propStartTime = maneuversDict["endTime"][currentManeuver]
                currentManeuver += 1
                # Uncomment this line if you want the satellite to spin for the entire propagation step
                # propEndTime = maneuversDict['startTime'][currentManeuver]
                propEndTime = min(propStartTime + 200, missionEndDate)
                newData = False
        else:
            isManeuver = False

        if isManeuver:  # Spin. +X->VNC_X constraint
            missionParams["q1"][index] = vncDf["q1"][index]
            missionParams["q2"][index] = vncDf["q2"][index]
            missionParams["q3"][index] = vncDf["q3"][index]
            missionParams["q4"][index] = vncDf["q4"][index]
            missionParams["wx"][index] = 0
            missionParams["wy"][index] = 0
            missionParams["wz"][index] = 0
            missionParams["bx"][index] = 0
            missionParams["by"][index] = 0
            missionParams["bz"][index] = 0
            propinitQuat = np.array(
                [
                    [
                        vncDf["q1"][index],
                        vncDf["q2"][index],
                        vncDf["q3"][index],
                        vncDf["q4"][index],
                    ]
                ]
            ).T

        else:  # Propagation. Nutation Damping. Spin.
            if not newData:
                totalIntegrationTime = (propEndTime - propStartTime) + 1
                print(f"Total Integration Time: {totalIntegrationTime}")
                (
                    q1,
                    q2,
                    q3,
                    q4,
                    omegax,
                    omegay,
                    omegaz,
                    biasx,
                    biasy,
                    biasz,
                ) = generateSyntheticData(
                    propinitQuat,
                    cameradt,
                    coldGasThrustKickTime,
                    coldGasKickDuration,
                    omegaInit,
                    biasInit,
                    1.0 / gyroSampleCount,
                    totalIntegrationTime,
                    gyroSigma,
                    gyroNoiseSigma,
                    integrationTimeStep=0.1,
                )
                newData = True
            # missionQuats['q1'][index] = q1(timestamp-propStartTime)
            # missionQuats['q2'][index] = q2(timestamp-propStartTime)
            # missionQuats['q3'][index] = q3(timestamp-propStartTime)
            # missionQuats['q4'][index] = q4(timestamp-propStartTime)
            currentTime = timestamp - propStartTime
            temp = np.array(
                [q1(currentTime), q3(currentTime), q3(currentTime), q4(currentTime)]
            )
            temp /= np.linalg.norm(temp)
            missionParams["q1"][index] = temp[0]
            missionParams["q2"][index] = temp[1]
            missionParams["q3"][index] = temp[2]
            missionParams["q4"][index] = temp[3]
            if timestamp <= propEndTime:
                missionParams["wx"][index] = omegax(currentTime)
                missionParams["wy"][index] = omegay(currentTime)
                missionParams["wz"][index] = omegaz(currentTime)
                missionParams["bx"][index] = biasx(currentTime)
                missionParams["by"][index] = biasy(currentTime)
                missionParams["bz"][index] = biasz(currentTime)
            else:
                # This is necessary because the interpolation function does not extend beyond the provided end date
                missionParams["wx"][index] = 0
                missionParams["wy"][index] = 0
                missionParams["wz"][index] = 0
                missionParams["bx"][index] = 0
                missionParams["by"][index] = 0
                missionParams["bz"][index] = 0

    return missionParams


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-d", "--opnavdataset", help="path to OpNav Dataset root folder")
    ap.add_argument(
        "-s",
        "--startendcsv",
        help="name of CSV file that contains start and end date to generate data (like root/startEndDates.csv)",
    )
    ap.add_argument(
        "-r",
        "--samplingRate",
        help="sampling rate of ephemeris/traj to seconds. Eg: if data is in 1-min interval, -r=60",
    )
    ap.add_argument(
        "-g", "--gyro_t", help="time delta (seconds) between each gyro measurement"
    )
    ap.add_argument(
        "-o",
        "--outfile",
        help="name of output csv file (include .csv); will be created in root/attitude/",
    )
    args = vars(ap.parse_args())

    INITIAL_TRAJ_PATH = os.path.join(
        args["opnavdataset"], "trajectory", "1min_stk_active_sampled_traj.csv"
    )
    INITIAL_VNC_PATH = os.path.join(
        args["opnavdataset"],
        "attitude",
        "1min_sampled_vncearth_stk_active_attitude.csv",
    )
    INITIAL_ATT_PATH = os.path.join(args["opnavdataset"], "attitude", args["outfile"])
    SAMPLED_MOON_PATH = os.path.join(
        args["opnavdataset"], "ephemeris", "1min_stk_active_sampled_moon_eph.csv"
    )
    SAMPLED_SUN_PATH = os.path.join(
        args["opnavdataset"], "ephemeris", "1min_stk_active_sampled_moon_eph.csv"
    )
    MANEUVER_CHECKPOINT_PATH = os.path.join(
        args["opnavdataset"], "maneuvers", "checkpoints.csv"
    )
    START_END_DATES_PATH = args[
        "startendcsv"
    ]  # os.path.join(args['opnavdataset'], 'startEndDates.csv')

    simSamplingRate = float(args["samplingRate"])

    moonDf = pd.read_csv(SAMPLED_MOON_PATH)
    # sunDf = pd.read_csv(SAMPLED_SUN_PATH)
    trajDf = pd.read_csv(INITIAL_TRAJ_PATH)
    vncDf = pd.read_csv(INITIAL_VNC_PATH)
    manCheckDf = pd.read_csv(MANEUVER_CHECKPOINT_PATH)
    startEndDatesDf = pd.read_csv(START_END_DATES_PATH)

    # Obtain mission timeline (in seconds)
    missionStartDate, missionEndDate = getMissionTimeline(startEndDatesDf)
    maneuversDict = extractCheckpoints(manCheckDf, missionEndDate)
    # print(missionStartDate, missionEndDate, simSamplingRate)
    missionTimeline = np.arange(
        missionStartDate, missionEndDate + simSamplingRate, simSamplingRate
    )

    # Obtain mission attitude data
    # Uncomment the following 3 lines to generate attitude data for simulation (eg. if you have a new dataset, if you want a different spin rate)...
    missionParams = createDiscreteAttitudeManeuvers(
        maneuversDict,
        vncDf,
        missionStartDate,
        missionEndDate,
        missionTimeline,
        float(args["gyro_t"]),
    )
    attDf = pd.DataFrame.from_dict(missionParams)
    attDf.to_csv(os.path.join(INITIAL_ATT_PATH), index=False)
