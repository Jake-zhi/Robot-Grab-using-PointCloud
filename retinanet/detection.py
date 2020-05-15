from keras_retinanet import models
from keras_retinanet.utils.image import read_image_bgr, preprocess_image, resize_image
from keras_retinanet.utils.visualization import draw_box, draw_caption
from keras_retinanet.utils.colors import label_color
from keras_retinanet.utils.gpu import setup_gpu

import cv2
import numpy as np
import time

def detect_init():
    # use this to change which GPU to use
    gpu = 0
    
    # set the modified tf session as backend in keras
    setup_gpu(gpu)
    # adjust this to point to your downloaded/trained model
    # models can be downloaded here: https://github.com/fizyr/keras-retinanet/releases
    # model_path = 'retinanet/BOP/resnet50_mydata_800_1333.h5'
    # model_path = 'BOP/resnet50_mydata_800_1333.h5'
    # model_path = 'retinanet/MYRGB/MYRGB_50.h5'
    model_path = 'MYRGB/MYRGB_50.h5'
    
    global label_2_name
    label_2_name = {0: 'M1', 1: 'M3', 2: 'M4', 3: 'M7', 4: 'M8', 5: 'M5', 6: 'M6', 7: 'M2'}
    # label_2_name = {0: 'duck', 1: 'pot', 2: 'cup', 3: 'drill'}
    
    # load retinanet model
    
    model = models.load_model(model_path)
    
    # if the model is not converted to an inference model, use the line below
    # see: https://github.com/fizyr/keras-retinanet#converting-a-training-model-to-inference-model
    # model = models.convert_model(model)
    
    # print(model.summary())
    return model

def detect(image, model, mode="runtime"):
    # copy to draw on
    draw = image.copy()

    # preprocess image for network
    image = preprocess_image(image)
    image, scale = resize_image(image)

    # process image
    start = time.time()
    boxes, scores, labels = model.predict_on_batch(np.expand_dims(image, axis=0))
    print("detection processing time: ", time.time() - start)
    if mode == 'runtime':
        boxes /= scale
        return boxes, scores, labels
    if mode == 'debug':
        # correct for image scale
        boxes /= scale

        # visualize detections
        for box, score, label in zip(boxes[0], scores[0], labels[0]):
            # scores are sorted so we can break
            if score < 0.8:
                break

            color = label_color(label)

            b = box.astype(int)
            draw_box(draw, b, color=color)
            caption = "{} {:.3f}".format(label_2_name[label], score)
            draw_caption(draw, b, caption)

        cv2.imshow("detect result", draw)
        cv2.waitKey()
        return boxes, scores, labels
    
if __name__ == "__main__":
    # img = cv2.imread("BOP/rgb.png")
    model = detect_init()
    for i in range(6):
    # i = 1072
        img = cv2.imread("D:\CODE&DATA\CODE\\robot_grasp\data\scene\MYDATA\\rgb/%s.png" % i)
        boxes, scores, labels = detect(img, model, "debug")
    
