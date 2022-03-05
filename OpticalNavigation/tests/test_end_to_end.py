import cv2
import unittest
from OpticalNavigation.tests import test_reprojections


class TestEndToEnd(unittest.TestCase):
    def get_images_and_reproject(self):

        gn_list, st_list = test_reprojections.get_images()
        gn_imgs, st_imgs, re_imgs = [], [], []

        for idx, gnName in enumerate(gn_list):
            src = cv2.imread(gn_list[idx])
            tgt = cv2.imread(st_list[idx])
            re_img, _ = test_reprojections.reproj(src, gnName)
            gn_imgs.append(src)
            st_imgs.append(tgt)
            re_imgs.append(re_img)

        return gn_imgs, st_imgs, re_imgs

    def test_end_to_end(self):
        gn_imgs, st_imgs, re_imgs = self.get_images_and_reproject()
        pass


if __name__ == "__main__":
    unittest.main()
