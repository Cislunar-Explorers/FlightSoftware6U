from opnav.core.preprocess import extract_frames
from opnav.core.sense import select_camera, record_video
from utils.constants import OPNAV_MEDIA_DIR
from opnav.core.const import CameraRecordingParameters
import utils.parameters as params

# from picamera import PiCamera
import argparse


def record_and_extract():
    recordings = []
    camera_rec_params = CameraRecordingParameters(
        params.CAMERA_FPS,
        params.CAMERA_RECORDING_TIME,
        params.CAMERA_LOW_EXPOSURE,
        params.CAMERA_HIGH_EXPOSURE,
    )

    # Record videos
    for i in [1, 2, 3]:  # These are the hardware IDs of the camera mux ports
        select_camera(id=i)

        print(f"Recording from camera {i}")
        # TODO: figure out exposure parameters
        vidData1 = record_video(
            OPNAV_MEDIA_DIR + f"cam{i}_expLow.mjpeg",
            framerate=camera_rec_params.fps,
            recTime=camera_rec_params.recTime,
            exposure=camera_rec_params.expLow,
        )
        vidData2 = record_video(
            OPNAV_MEDIA_DIR + f"cam{i}_expHigh.mjpeg",
            framerate=camera_rec_params.fps,
            recTime=camera_rec_params.recTime,
            exposure=camera_rec_params.expHigh,
        )

        print(f"vidData1: {vidData1}")
        print(f"vidData2: {vidData2}")
        recordings.append(vidData1)
        recordings.append(vidData2)

    frames0 = extract_frames(
        vid_dir=recordings[0][0],
        timestamps=recordings[0][1],
        cameraRecParams=camera_rec_params,
    )
    print(f"frames0 size: {len(frames0)}")
    print(frames0)
    print("\n")
    frames1 = extract_frames(
        vid_dir=recordings[1][0],
        timestamps=recordings[1][1],
        cameraRecParams=camera_rec_params,
    )
    print(f"frames1 size: {len(frames1)}")
    print(frames1)
    print("\n")
    frames2 = extract_frames(
        vid_dir=recordings[2][0],
        timestamps=recordings[2][1],
        cameraRecParams=camera_rec_params,
    )
    print(f"frames2 size: {len(frames2)}")
    print(frames2)
    print("\n")
    frames3 = extract_frames(
        vid_dir=recordings[3][0],
        timestamps=recordings[3][1],
        cameraRecParams=camera_rec_params,
    )
    print(f"frames3 size: {len(frames3)}")
    print(frames3)
    print("\n")
    frames4 = extract_frames(
        vid_dir=recordings[4][0],
        timestamps=recordings[4][1],
        cameraRecParams=camera_rec_params,
    )
    print(f"frames4 size: {len(frames4)}")
    print(frames4)
    print("\n")
    frames5 = extract_frames(
        vid_dir=recordings[5][0],
        timestamps=recordings[5][1],
        cameraRecParams=camera_rec_params,
    )
    print(f"frames5 size: {len(frames5)}")
    print(frames5)
    print("\n")
    # frames = frames0 + frames1 + frames2 + frames3 + frames4 + frames5


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-m", "--mode", help="Restart mode for camera mux or regular run")
    args = vars(ap.parse_args())

    if args["mode"] == "frames":
        record_and_extract()
