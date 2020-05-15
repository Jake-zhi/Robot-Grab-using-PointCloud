import cv2
import numpy as np

def generate_depth_mask(box, rgb_img_shape, depth_mask_shape):
    mask_img = np.zeros(rgb_img_shape[0:2], dtype=np.uint8)
    b = box.astype(int)
    cv2.rectangle(mask_img, (b[0], b[1]), (b[2], b[3]), 255, cv2.FILLED, cv2.LINE_AA)
    mask_img = cv2.resize(mask_img, (depth_mask_shape[1], depth_mask_shape[0]))
    return mask_img

def copy_img_mask(depth_img, mask):
    new_img = cv2.copyTo(depth_img, mask.maskimg)
    return new_img
if __name__ == '__main__':
    copy_img_mask()