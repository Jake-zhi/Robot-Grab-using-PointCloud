import glob
import cv2

images = glob.glob('chease_board/*.png')
for fname in images:
    img = cv2.imread(fname)
    new = cv2.resize(img, (640, 480), interpolation=cv2.INTER_AREA)
    # cv2.imshow("new", new)
    # cv2.waitKey(0)
    cv2.imwrite(fname+".png", new)