from .video_capture import *
from .img_preprocess import *

# Current camera mux ports in use: 1, 3, 4

# Test 1: Single camera record
def singleCamRec():
    # Choose first camera mux port
    selectCamera(1)
    # Record video
    img = rawObservation('single_recording.mjpeg')
    print(f"End timestamp: {img['single_recording.mjpeg']}")
    # Check file system after


# Test 2: Single camera record modified framerate
def singleCamRecFramerate():
    # Choose first camera mux port
    selectCamera(1)
    # Record video
    img = rawObservation('single_rec_30fps.mjpeg', 30)
    print(f"End timestamp: {img['single_rec_30fps.mjpeg']}")
    # Check file system after


# Test 3: Multiplex cameras
def multiCamRec():
    # Choose first camera mux port
    selectCamera(1)
    # Record video
    img = rawObservation('mux_rec_1.mjpeg')
    print(f"End timestamp 1: {img['mux_rec_1.mjpeg']}")

    # Switch mux
    selectCamera(3)
    # Record video
    rawObservation('mux_rec_2.mjpeg')
    print(f"End timestamp 2: {img['mux_rec_2.mjpeg']}")

    # Switch mux
    selectCamera(4)
    # Record video
    rawObservation('mux_rec_3.mjpeg')
    print(f"End timestamp 3: {img['mux_rec_3.mjpeg']}")
    # Check file system after


# Test 4: Image preprocess
def imgPreproc():
    # Choose first camera mux port
    selectCamera(3)
    # Record video
    rawObservation('preproc_b4.mjpeg')

    # Send to image preprocessing
    vid = cv2.VideoCapture('preproc_b4.mjpeg')
    i = 1
    while (vid.isOpened()):
        retval, img = vid.read()
        if retval == True:
            cv2.imwrite(f'preproc_b4_F{i}.jpg', img)
            preprocess(f'preproc_b4_F{i}.jpg', f'preproc_after_F{i}.jpg')
            i += 1
        else:
            break
    vid.release()
    # Compare images


# Test 5: Image preprocess multiple videos
def multiImgPreproc():
    # Choose first camera mux port
    selectCamera(1)
    # Record video
    rawObservation('pre_mux_1_b4.mjpeg')
    # Send to image preprocessing
    vid = cv2.VideoCapture('pre_mux_1_b4.mjpeg')
    i = 1
    while (vid.isOpened()):
        retval, img = vid.read()
        if retval == True:
            cv2.imwrite(f'pre_mux_1_b4_F{i}.jpg', img)
            preprocess(f'pre_mux_1_b4_F{i}.jpg', f'preproc_1_after_F{i}.jpg')
            i += 1
        else:
            break
    vid.release()

    # Switch mux
    selectCamera(3)
    # Record video
    rawObservation('pre_mux_2_b4.mjpeg')
    # Send to image preprocessing
    vid = cv2.VideoCapture('pre_mux_2_b4.mjpeg')
    i = 1
    while (vid.isOpened()):
        retval, img = vid.read()
        if retval == True:
            cv2.imwrite(f'pre_mux_2_b4_F{i}.jpg', img)
            preprocess(f'pre_mux_2_b4_F{i}.jpg', f'preproc_2_after_F{i}.jpg')
            i += 1
        else:
            break
    vid.release()

    # Switch mux
    selectCamera(4)
    # Record video
    rawObservation('pre_mux_3_b4.mjpeg')
    # Send to image preprocessing
    vid = cv2.VideoCapture('pre_mux_3_b4.mjpeg')
    i = 1
    while (vid.isOpened()):
        retval, img = vid.read()
        if retval == True:
            cv2.imwrite(f'pre_mux_3_b4_F{i}.jpg', img)
            preprocess(f'pre_mux_3_b4_F{i}.jpg', f'preproc_3_after_F{i}.jpg')
            i += 1
        else:
            break
    vid.release()
    # Compare images


if __name__ == "__main__":
    singleCamRec()
