from pathlib import Path
import numpy as np
import cv2

class OpticalFlow:
    @staticmethod
    def get_displacement_px(img1, img2) -> tuple:
        """
        Uses Lucas-Kanade to calculate the (dx, dy) pixel displacement of the camera between two images.
        Args:
            img1 (str path or ndarray): First image.
            img2 (str path or ndarray): Second image.
        
        Returns:
            dx, dy (tuple): the displacement in pixels between the two images.
        """
        img1 = OpticalFlow.convert_to_grayscale(img1)
        img2 = OpticalFlow.convert_to_grayscale(img2)

        p0 = cv2.goodFeaturesToTrack(img1, mask=None, **feature_params)

        # paras for Shi-Tomasi corner detection (finding reliable points to track)
        feature_params = dict(maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)

        # params for Lucas-Kanade
        lk_params = dict(
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
        )
        return dx, dy

    @staticmethod
    def convert_to_grayscale(img):
        """
        Uses Lucas-Kanade to calculate the (dx, dy) pixel displacement of the camera between two images.
        Args:
            img1 (str path or ndarray): First image.
            img2 (str path or ndarray): Second image.
        
        Returns:
            img_grayscale (ndarray): the displacement in pixels between the two images.
        """
        if (isinstance(img, str)):
            img_grayscale = cv2.imread(img)
            if img_grayscale==None:
                raise ValueError("File path is not valid image.")
            return img_grayscale
        if (isinstance(img, np.ndarray)):
            img_grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            return img_grayscale

        raise ValueError("Image is not a valid filepath or numpy array.")
        