import cv2 as cv
import numpy as np
import sys

print("Lettuce Harvest")

# Load the image and display it
image = cv.imread(sys.argv[1])

# if a pixel is white, turn it black
def white_to_black(image):
    for i in range(image.shape[0]):
        for j in range(image.shape[1]):
            if image[i][j][0] >= 230 and image[i][j][1] >= 230 and image[i][j][2] >= 230:
                image[i][j][0] = 0
                image[i][j][1] = 0
                image[i][j][2] = 0
    return image

image = white_to_black(image)
# separate the image into its channels
(B, G, R) = cv.split(image)

# filter

G[G < 60] = 0

# refine the mask
element = cv.getStructuringElement(cv.MORPH_RECT, (2,2))
mask = cv.erode(G, element, iterations = 3)
G = cv.medianBlur(G, 5)
G[G > 60] = 255
G = cv.dilate(G , element, iterations = 10)

# display the green channel
#cv.imshow("Green", G)

params = cv.SimpleBlobDetector_Params()
params.minDistBetweenBlobs = 10

params.filterByColor = False 
#params.minThreshold = 255
#params.maxThreshold = 255

params.filterByArea = True
params.maxArea = 1e40
params.minArea = 1e4

params.filterByCircularity = False
params.minCircularity = 0

params.filterByConvexity = False
params.minConvexity = 0

params.filterByInertia = False
params.minInertiaRatio = 0.1


detector = cv.SimpleBlobDetector_create(params)
keypoints = detector.detect(G)

#blank = np.zeros((1, 1))
blobs = cv.drawKeypoints(G, keypoints, np.array([]), (0, 0, 255),cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

cv.imshow("Blobs Using Area", blobs)
# bring the image to the front and right of the screen


#im_with_keypoints = cv.drawKeypoints(G, keypoints, np.array([]), (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
## Show keypoints
#cv.imshow("Keypoints", im_with_keypoints)
cv.waitKey(0)
cv.destroyAllWindows()