# Standard imports
import cv2
import numpy as np
import argparse

# import copy
from core.find_algos.find_with_kmeans import getkmeans

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--image", help="path to the image")
    args = vars(ap.parse_args())

    # Read image
    # imOrig = cv2.imread(os.path.join("D:", "OpNav", "data", "EclipseAndCrescentImages", "images", "EarthMoon.jpg"),
    #     cv2.IMREAD_GRAYSCALE,
    # )
    imOrig = cv2.imread(args["image"], cv2.IMREAD_GRAYSCALE)
    cv2.namedWindow("original", cv2.WINDOW_NORMAL)
    cv2.imshow("original", imOrig)
    cv2.waitKey(0)

    # im = cv2.imread("blob.jpg", cv2.IMREAD_GRAYSCALE)

    for minArea in range(0, 100, 10):
        print(f"Trying MinArea: {minArea}")
        im = imOrig.copy()

        im = getkmeans(im)
        thresh = 0  # 245
        im = cv2.threshold(im[:, :, 0], thresh, 255, cv2.THRESH_BINARY)[1]
        im = cv2.medianBlur(im, 11)
        cv2.namedWindow("Segmented", cv2.WINDOW_NORMAL)
        cv2.imshow("Segmented", im)
        cv2.waitKey(0)

        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        params.minThreshold = 0
        params.maxThreshold = 255

        # Filter by Area.
        params.filterByArea = True
        params.minArea = minArea
        params.maxArea = 10000

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.1

        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.01

        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.01

        # Set up the detector with default parameters.
        detector = cv2.SimpleBlobDetector_create(params)
        print("detector ready")

        # Detect blobs.
        keypoints = detector.detect(im)

        if len(keypoints) == 0:
            continue

        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(
            im,
            keypoints,
            np.array([]),
            (0, 0, 255),
            cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
        )

        # Show keypoints
        cv2.namedWindow("Keypoints", cv2.WINDOW_NORMAL)
        cv2.imshow("Keypoints", im_with_keypoints)
        cv2.waitKey(0)
        print(f"minArea: {minArea}")

    print("done")
