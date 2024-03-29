# Find Algorithms
This folder contains all the implementations of the find algorithm that have been worked on over the years. Below is a description of each of the find algorithms and their status.<br><br>

## Current Implementation in Use: <code>find_with_contours.py</code><br><br>

## <code>find.py</code><br>
Status: 🟥Not viable<br>
Primary Developer: Sean Kumar<br>
Description: This was the first find algorithm developed which utilizes the hough circle transform for detection. First the sun is searched for in the image, and if found, is erased from the image. This modified image is then used for Earth detection, and a similar process is done for altering the image to be used for moon detection. Unfortunately, the computational constraints of the Raspberry Pi 1A+ meant that the hough transform operation was too memory intensive to function, leading this algorithm (more importantly, the hough transform on full 4K images) to be a non-viable option.

## <code>find_with_binary_thresh.py</code><br>
Status: 🟥Experimental, potentially unviable<br>
Primary Developer: Adam Nasir<br>
Description: Circle detection algorithm done in a similar way to find_with_contours. Uses the Hough transform, in original performance-inhibiting way.

## <code>find_with_blobs.py</code><br>
Status: ⬜Experimental, untested<br>
Primary Developer: Sean Kumar<br>
Description: Not too sure about this one. Uses find_with_kmeans to get a binary image of detected bodies, and also detects/circles blobs in these contours. These blobs seem to be places where misclassifications are occuring within a contour.

## <code>find_with contrours.py</code><br>
Status: 🟩Potentially viable<br>
Primary Developer(s): Stephen Zakoworotny, Dr.Muhlberger, Matthew Hall-Pena<br>
Description: Find implementation that takes into account camera characteristics, as well as rolling shutter. Performs transofmrations from pixel coordinates to gnominic projection to spherical coordiantes, and back to the stereographic projection and back to pixel coordinates. Returns results as an ImageDetectionCricles object ([opnav/core/const.py](https://github.com/Cislunar-Explorers/FlightSoftware/blob/master/opnav/core/const.py#L205)), which encapsulates the center and size information for each body.

## <code>find_with_hough_transform_and_contours.py</code><br>
Status: 🟩Potentially viable<br>
Primary Developer: Andrew Xu<br>
Description: Performs circle detection by using the Hough transform on a small region of interest (for efficiency, as well as accuracy), while defaulting to the find_with_contours implementation if the Hough transform does not find any contours.

## <code>find_with_kmeans.py</code><br>
Status: ⬜Experimental, tested<br>
Primary Developer: Sean Kumar<br>
Description: Uses k-means clustering to classify pixels as either being part of a body or not. Could be helpful in classifying bodies.

## <code>find_with_threepoint.py</code><br>
Status: 🟩Experimental, potentially viable<br>
Primary Developer: Adam Nasir<br>
Description: Performs circle detection using three points which lie on the edge of a circle.
