import cv2
import numpy as np
import glob

images = []
files = glob.glob("*.jpg")
for myFile in files:
	print(myFile)
	image = cv2.imread(myFile)
	std = np.std(image)
	images.append(image)
	print(std)
	print(np.max(image))
	

