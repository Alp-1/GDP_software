import time

import cv2
from skimage import data, segmentation, feature, future
from functools import partial
from joblib import dump, load
from joblib import dump, load

sigma_min = 1
sigma_max = 32


# 4x downsampling
downsampled_width = 212
downsampled_height = 120


class SemanticSegmentation:
    def __init__(self, model_path='model_10.joblib'):
        self.model = load(model_path)

    def extract_features(self,img):
        img_new = cv2.resize(img, (downsampled_width, downsampled_height))
        features_func = partial(feature.multiscale_basic_features,
                                intensity=True, edges=True, texture=True,
                                sigma_min=sigma_min, sigma_max=sigma_max,
                                channel_axis=-1)
        return features_func(img_new)

    def get_semantic_map(self, img):
        start_time = time.time()
        features = self.extract_features(img)
        result = future.predict_segmenter(features, self.model)
        return result

