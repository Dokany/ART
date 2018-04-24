import cv2
import numpy as np

    # Currently Used one
def segment_color(img, lower, upper):
    """
    :param img: Image to isolate teh color of
    :param lower: [lowerHue, lowerSat, lowerVal]
    :param upper: [upperHue, upperSat, upperVal]
    :return: Isolated image
    """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    if lower[0] > upper[0]:
        # If the HSV values wrap around, then intelligently mask it

        upper1 = [180, upper[1], upper[2]]
        mask1 = cv2.inRange(hsv, np.array(lower), np.array(upper1))

        lower2 = [0, lower[1], lower[2]]
        mask2 = cv2.inRange(hsv, np.array(lower2), np.array(upper))

        mask = mask1 + mask2

    else:
        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))

    final = cv2.bitwise_and(img, img, mask=mask)
    rGray = cv2.cvtColor(final, cv2.COLOR_BGR2GRAY)
    ret, rThresh = cv2.threshold(rGray, 20, 255, cv2.THRESH_BINARY)
    cv2.imshow("Mask", rThresh)
    # return final

def nothing(x):
    pass


HSV_image = "HSV image"
cv2.namedWindow("HSV image", cv2.WINDOW_NORMAL)
cv2.createTrackbar('Low_H', HSV_image, 0, 255, nothing)
cv2.createTrackbar('Low_S', HSV_image, 0, 255, nothing)
cv2.createTrackbar('Low_V', HSV_image, 0, 255, nothing)
cv2.createTrackbar('High_H', HSV_image, 0, 255, nothing)
cv2.createTrackbar('High_S', HSV_image, 0, 255, nothing)
cv2.createTrackbar('High_V', HSV_image, 0, 255, nothing)
# cv2.namedWindow("thresh image", cv2.WINDOW_AUTOSIZE)

vs = cv2.VideoCapture(0)
# vs.set(cv2.CAP_PROP_BRIGHTNESS, 255)
# vs.set(cv2.CAP_PROP_AUTOFOCUS, 1)
vs.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
vs.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
_, original_image = vs.read()

while True:

    low_H = cv2.getTrackbarPos("Low_H", HSV_image)
    low_S = cv2.getTrackbarPos("Low_S", HSV_image)
    low_V = cv2.getTrackbarPos("Low_V", HSV_image)
    High_H = cv2.getTrackbarPos("High_H", HSV_image)
    High_S = cv2.getTrackbarPos("High_S", HSV_image)
    High_V = cv2.getTrackbarPos("High_V", HSV_image)
    lower_color = (low_H, low_S, low_V)
    upper_color = (High_H, High_S, High_V)
    try:

        _, original_image = vs.read()
        segment_color(original_image, lower_color, upper_color)

        cv2.imshow("Original Image", original_image)

        # grey_image = cv2.cvtColor(vars.original_image, cv2.COLOR_RGB2GRAY)

        key = cv2.waitKey(1)
        if key == 27:  # exit on ESC
            break
    except KeyboardInterrupt:
        break

vs.stop()
cv2.destroyAllWindows()
