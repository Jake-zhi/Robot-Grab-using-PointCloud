import cv2
import numpy as np

class mask:
    def __init__(self, mask_img, score, label):
        self.maskimg = mask_img
        self.score = score
        self.label = label
        self.shape = mask_img.shape


def generate_depth_mask(box, score, label, rgb_img_shape, depth_mask_shape):
    draw = np.zeros(rgb_img_shape[0:2], dtype=np.uint8)
    b = box.astype(int)
    cv2.rectangle(draw, (b[0], b[1]), (b[2], b[3]), 255, cv2.FILLED, cv2.LINE_AA)
    draw = cv2.resize(draw, (depth_mask_shape[1], depth_mask_shape[0]))
    return mask(draw, score, label)

def copy_img_mask(depth_img, mask):
    new_img = cv2.copyTo(depth_img, mask.maskimg)
    return new_img
if __name__ == '__main__':
    copy_img_mask()