from OpticalNavigation.core.preprocess import extract_frames
from OpticalNavigation.core.sense import select_camera, record_video
from utils.constants import OPNAV_MEDIA_PATH
from OpticalNavigation.core.const import CameraRecordingParameters, ImageDetectionCircles
from OpticalNavigation.core.opnav import __get_elapsed_time
import utils.parameters as params
from picamera import PiCamera
import time
import numpy as np
from datetime import datetime, timedelta, timezone
import argparse


def record_and_extract():
    recordings = []
    camera_rec_params = CameraRecordingParameters(params.CAMERA_FPS, params.CAMERA_RECORDING_TIME, params.CAMERA_LOW_EXPOSURE, params.CAMERA_HIGH_EXPOSURE)

    # Record videos
    for i in [1, 2, 3]: # These are the hardware IDs of the camera mux ports
        select_camera(id = i)

        print(f"Recording from camera {i}")
        # TODO: figure out exposure parameters
        fileDiffTime1 = record_video(OPNAV_MEDIA_PATH + f"cam{i}_expLow.mjpeg", framerate = camera_rec_params.fps, recTime=camera_rec_params.recTime, exposure=camera_rec_params.expLow)
        fileDiffTime2 = record_video(OPNAV_MEDIA_PATH + f"cam{i}_expHigh.mjpeg", framerate = camera_rec_params.fps, recTime=camera_rec_params.recTime, exposure=camera_rec_params.expHigh)

        print(f"fileDiffTime1: {fileDiffTime1}")
        print(f"fileDiffTime2: {fileDiffTime2}")
        recordings.append(fileDiffTime1)
        recordings.append(fileDiffTime2)

    frames0 = extract_frames(vid_dir=recordings[0][0], frameDiff=recordings[0][1], endTimestamp=recordings[0][2], cameraRecParams=camera_rec_params)
    print(f"frames0 size: {len(frames0)}")
    print(frames0 + "\n")
    frames1 = extract_frames(vid_dir=recordings[1][0], frameDiff=recordings[1][1], endTimestamp=recordings[1][2], cameraRecParams=camera_rec_params)
    print(f"frames1 size: {len(frames1)}")
    print(frames1 + "\n")
    frames2 = extract_frames(vid_dir=recordings[2][0], frameDiff=recordings[2][1], endTimestamp=recordings[2][2], cameraRecParams=camera_rec_params)
    print(f"frames2 size: {len(frames2)}")
    print(frames2 + "\n")
    frames3 = extract_frames(vid_dir=recordings[3][0], frameDiff=recordings[3][1], endTimestamp=recordings[3][2], cameraRecParams=camera_rec_params)
    print(f"frames3 size: {len(frames3)}")
    print(frames3 + "\n")
    frames4 = extract_frames(vid_dir=recordings[4][0], frameDiff=recordings[4][1], endTimestamp=recordings[4][2], cameraRecParams=camera_rec_params)
    print(f"frames4 size: {len(frames4)}")
    print(frames4 + "\n")
    frames5 = extract_frames(vid_dir=recordings[5][0], frameDiff=recordings[5][1], endTimestamp=recordings[5][2], cameraRecParams=camera_rec_params)
    print(f"frames5 size: {len(frames5)}")
    print(frames5 + "\n")
    frames = frames0 + frames1 + frames2 + frames3 + frames4 + frames5

def unix_timestamp_test():
    observeStart = datetime(2020, 7, 28, 22, 8, 3)  # TEMPORARY TESTING START TIME
    observeStart = int(observeStart.replace(tzinfo=timezone.utc).timestamp() * 10 ** 6)  # In unix time

    recordings = []
    camera_rec_params = CameraRecordingParameters(params.CAMERA_FPS, params.CAMERA_RECORDING_TIME, params.CAMERA_LOW_EXPOSURE, params.CAMERA_HIGH_EXPOSURE)
    timeDeltaAvgs = [0, 0, 0]

    for i in [1, 2, 3]: # These are the hardware IDs of the camera mux ports
        select_camera(id = i)

        # Get Unix time before recording(in seconds floating point -> microseconds)
        # Get camera time (in microseconds)
        linuxTime1:int
        cameraTime1:int
        with PiCamera() as camera:
            linuxTime1 = int(time.time() * 10 ** 6)
            cameraTime1 = camera.timestamp
        # Get difference between two clocks
        timeDelta1 = linuxTime1 - cameraTime1

        print(f"Recording from camera {i}")
        # TODO: figure out exposure parameters
        fileDiffTime1 = record_video(OPNAV_MEDIA_PATH + f"cam{i}_expLow.mjpeg", framerate = camera_rec_params.fps, recTime=camera_rec_params.recTime, exposure=camera_rec_params.expLow)
        fileDiffTime2 = record_video(OPNAV_MEDIA_PATH + f"cam{i}_expHigh.mjpeg", framerate = camera_rec_params.fps, recTime=camera_rec_params.recTime, exposure=camera_rec_params.expHigh)

        # Get Unix time after recording(in seconds floating point -> microseconds)
        # Get camera time (in microseconds)
        linuxTime2:int
        cameraTime2:int
        with PiCamera() as camera:
            linuxTime2 = time.time() * 10 ** 6
            cameraTime2 = camera.timestamp
        # Get difference between two clocks
        timeDelta2 = linuxTime2 - cameraTime2

        timeDeltaAvg = (timeDelta1 + timeDelta2) / 2
        timeDeltaAvgs[i-1] = timeDeltaAvg

        recordings.append(fileDiffTime1)
        recordings.append(fileDiffTime2)

    frames0 = extract_frames(vid_dir=recordings[0][0], frameDiff=recordings[0][1], endTimestamp=recordings[0][2], cameraRecParams=camera_rec_params)
    print(f"frames0 size: {len(frames0)}")
    print(frames0 + "\n")
    frames1 = extract_frames(vid_dir=recordings[1][0], frameDiff=recordings[1][1], endTimestamp=recordings[1][2], cameraRecParams=camera_rec_params)
    print(f"frames1 size: {len(frames1)}")
    print(frames1 + "\n")
    frames2 = extract_frames(vid_dir=recordings[2][0], frameDiff=recordings[2][1], endTimestamp=recordings[2][2], cameraRecParams=camera_rec_params)
    print(f"frames2 size: {len(frames2)}")
    print(frames2 + "\n")
    frames3 = extract_frames(vid_dir=recordings[3][0], frameDiff=recordings[3][1], endTimestamp=recordings[3][2], cameraRecParams=camera_rec_params)
    print(f"frames3 size: {len(frames3)}")
    print(frames3 + "\n")
    frames4 = extract_frames(vid_dir=recordings[4][0], frameDiff=recordings[4][1], endTimestamp=recordings[4][2], cameraRecParams=camera_rec_params)
    print(f"frames4 size: {len(frames4)}")
    print(frames4 + "\n")
    frames5 = extract_frames(vid_dir=recordings[5][0], frameDiff=recordings[5][1], endTimestamp=recordings[5], cameraRecParams=camera_rec_params)
    print(f"frames5 size: {len(frames5)}")
    print(frames5 + "\n")
    frames = frames0 + frames1 + frames2 + frames3 + frames4 + frames5

    earthDetection = np.array([0.0229743, 0.28930023, 0.95696267, 0.06673563], dtype=float)
    bestEarthTuple = (OPNAV_MEDIA_PATH + "cam3_expHigh_f17_t11585.jpg", 0.2902110283408872, earthDetection)

    moonDetection = np.array([0.17976721,-0.22221176,0.95828267,0.00805734], dtype=float)
    bestMoonTuple = (OPNAV_MEDIA_PATH + "cam3_expHigh_f17_t11585.jpg", 0.28582217699623985, moonDetection)

    sunDetection = np.array([-0.03859251,-0.74174568,0.66956998,0.02205351], dtype=float)
    bestSunTuple = (OPNAV_MEDIA_PATH + "cam2_expLow_f0_t2094.jpg", 0.21252515705861857, sunDetection)

    earthElapsed = __get_elapsed_time(bestEarthTuple, timeDeltaAvgs, observeStart)
    moonElapsed = __get_elapsed_time(bestMoonTuple, timeDeltaAvgs, observeStart)
    sunElapsed = __get_elapsed_time(bestSunTuple, timeDeltaAvgs, observeStart)

    print(f"Earth Elapsed = {earthElapsed}")
    print(f"Moon Elapsed = {moonElapsed}")
    print(f"Sun Elapsed = {sunElapsed}")

    print("Earth Elapsed Actual = 11.585")
    print("Moon Elapsed Actual = 11.585")
    print("Sun Elapsed Actual = 2.094")

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-m", "--mode", help="Restart mode for camera mux or regular run")
    args = vars(ap.parse_args())

    if args["mode"] == "frames":
        record_and_extract()
    elif args["mode"] == "timestamp":
        unix_timestamp_test()



