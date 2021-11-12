# https://www.thepythoncode.com/article/kmeans-for-image-segmentation-opencv-python
# Standard imports
import cv2
import numpy as np
import argparse


def getkmeans(imOrig):
    image = cv2.cvtColor(imOrig, cv2.COLOR_BGR2RGB)
    # image = cv2.medianBlur(imTemp, 5)
    # reshape the image to a 2D array of pixels and 3 color values (RGB)
    pixel_values = image.reshape((-1, 3))
    # convert to float
    pixel_values = np.float32(pixel_values)

    # define stopping criteria
    max_iters = 100
    epsilon = 10  # amount by which the clusters move
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, max_iters, epsilon)

    k = 2
    _, labels, (centers) = cv2.kmeans(
        pixel_values, k, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS
    )

    # convert back to 8 bit values
    centers = np.uint8(centers)

    # flatten the labels array
    # labels = labels.flatten()

    # color = np.copy(centers)
    # for r in range(color.shape[0]):
    #     color[r] = np.random.randint(255, size=3)

    # convert all pixels to the color of the centroids
    segmented_image = centers[labels.flatten()]

    # reshape back to the original image dimension
    segmented_image = segmented_image.reshape(image.shape)
    return segmented_image


if __name__ == "__main__":
    # Read image
    # imOrig = cv2.imread(os.path.join("D:", "OpNav", "data", "EclipseAndCrescentImages", "images", "EarthMoon.jpg"))
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--image", help="path to the image")
    args = vars(ap.parse_args())

    imOrig = cv2.imread(args["image"])
    cv2.imshow("original", imOrig)
    cv2.waitKey(0)
    segmented_image = getkmeans(imOrig)
    cv2.imshow("Segmented Image", segmented_image)
    cv2.waitKey(0)


# print(labels.shape)
# for l in range(k):
#     flat_segm_img = np.copy(image)
#     flat_segm_img[labels == l] = [255, 255, 255]
#     flat_segm_img = flat_segm_img.reshape(image.shape).astype(np.uint8)
#     cv2.imshow("Segmented Image " + str(l), flat_segm_img)
#     cv2.waitKey(0)
