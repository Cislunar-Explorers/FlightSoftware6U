# Find Algorithms
This folder contains all the implementations of the find algorithm that have been worked on over the years. Below is a description of each of the find algorithms and their status.<br><br>

## Current Implementation in Use
<code>find_with_contours.py</code><br><br>

## <code>find.py</code><br>
Status: 🟥Not viable<br>
Primary Developer: Sean Kumar<br>
Description: This was the first find algorithm developed which utilizes the hough circle transform for detection. First the sun is searched for in the image, and if found, is erased from the image. This modified image is then used for Earth detection, and a similar process is done for altering the image to be used for moon detection. Unfortunately, the computational constraints of the Raspberry Pi 1A+ meant that the hough transform operation was too memory intensive to function, leading this algorithm (more importantly, the hough transform on full 4K images) to be a non-viable option.

## <code>find_with_binary_thresh.py</code><br>
Status: 🟩Experimental, potentially viable<br>
Primary Developer: Adam Nasir<br>
Description:

## <code>find_with_blobs.py</code><br>
Status: 🟥Experimental, untested<br>
Primary Developer: Sean Kumar<br>
Description:

## <code>find_with contrours.py</code><br>
Status: 🟩Potentially viable<br>
Primary Developer: Stephen Zakoworotny, Dr.Muhlberger, Matthew Hall-Pena<br>
Description:

## <code>find_with_hough_transform_and_contours.py</code><br>
Status: 🟩Potentially viable<br>
Primary Developer: Andrew Xu<br>
Description:

## <code>find_with_kmeans.py</code><br>
Status: 🟥Experimental, untested<br>
Primary Developer: Sean Kumar<br>
Description:

## <code>find_with_threepoint.py</code><br>
Status: 🟩Experiemntal, potentially viable<br>
Primary Developer: Adam Nasir<br>
Description:
