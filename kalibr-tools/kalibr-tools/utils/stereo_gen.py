import time
import cv2 as cv
import numpy as np


def stereoPhotometicAlign(img_l, img_r):
    mean_l, mean_r = np.mean(img_l), np.mean(img_r)
    img_r = img_r * mean_l / mean_r
    img_r = np.clip(img_r, 0, 255).astype(np.uint8)
    return img_l, img_r


class StereoGen:
    def __init__(
            self,
            undist_l,
            undist_r,
            cam_idx_a=0,
            cam_idx_b=0,
            idx_l=1,
            idx_r=0,
            hitnet_model=None,
            is_rgb=False):
        self.undist_l = undist_l
        self.undist_r = undist_r
        self.idx_l = idx_l
        self.idx_r = idx_r
        self.cam_idx_a = cam_idx_a
        self.cam_idx_b = cam_idx_b
        self.extrinsic = self.extrinsicL()
        if hitnet_model is not None:
            self.enable_hitnet = True
            self.hitnet = hitnet_model
        else:
            self.enable_hitnet = False
        self.is_rgb = is_rgb

    def initRectify(self, K1, D1, K2, D2, size, R, T):
        self.R1, self.R2, self.P1, self.P2, self.Q, self.roi_l, self.roi_r = cv.stereoRectify(
            K1, D1, K2, D2, size, R, T)
        self.mapl0, self.mapl1 = cv.initUndistortRectifyMap(
            K1, D1, self.R1, self.P1, size, cv.CV_32FC1)
        self.mapr0, self.mapr1 = cv.initUndistortRectifyMap(
            K2, D2, self.R2, self.P2, size, cv.CV_32FC1)
        print("mapl0", self.mapl0)
        print("mapl1", self.mapl1)

    def genStereo(self, img_l, img_r):
        img_l = self.undist_l.undist(img_l, self.idx_l)
        img_r = self.undist_r.undist(img_r, self.idx_r)
        return img_l, img_r

    def rectifyL(self, img):
        img = self.undist_l.undist(img, self.idx_l)
        return cv.remap(img, self.mapl0, self.mapl1, cv.INTER_LINEAR)

    def extrinsicL(self):
        return self.undist_l.getPinholeCamExtrinsic(self.idx_l)

    def genRectStereo(self, img_l, img_r):
        s_img_l, s_img_r = self.genStereo(img_l, img_r)
        r_img_l = cv.remap(s_img_l, self.mapl0, self.mapl1, cv.INTER_LINEAR)
        r_img_r = cv.remap(s_img_r, self.mapr0, self.mapr1, cv.INTER_LINEAR)
        cv.imshow("rectified", cv.hconcat([r_img_l, r_img_r]))
        return r_img_l, r_img_r

    def genDisparity(self, img_l, img_r, max_disp=64, block_size=5):
        if not self.is_rgb:
            if len(img_l.shape) > 2 and img_l.shape[2] == 3:
                img_l = cv.cvtColor(img_l, cv.COLOR_BGR2GRAY)
            if len(img_r.shape) > 2 and img_r.shape[2] == 3:
                img_r = cv.cvtColor(img_r, cv.COLOR_BGR2GRAY)
        img_l, img_r = self.genRectStereo(img_l, img_r)
        print("img_l shape", img_l.shape[0], img_l.shape[1])
        cv.imshow("rectified", cv.hconcat([img_l, img_r]))
        if self.enable_hitnet:
            s = time.time()
            disparity = self.hitnet(img_l, img_r)
            dt = time.time() - s
            print("CNN Inference time {}ms".format(
                dt * 1000))
        else:
            stereo = cv.StereoSGBM.create(
                minDisparity=0,
                numDisparities=max_disp,
                blockSize=block_size,
                P1=8 * 3 * block_size ** 2, P2=32 * 3 * block_size ** 2,
                disp12MaxDiff=2,
                uniquenessRatio=10,
                speckleWindowSize=100,
                speckleRange=2
            )
            disparity = stereo.compute(img_l, img_r)
        stereo_img = cv.hconcat([img_l, img_r])
        cv.imshow("stereo", stereo_img)
        return disparity

    def genPointCloud(
            self, img_l, img_r, min_z=0.3, max_z=10, img_raw=None, enable_texture=True):

        disparity = self.genDisparity(img_l, img_r)
        if not self.enable_hitnet:
            disparity = disparity.astype(np.float32) / 16.0
        points = cv.reprojectImageTo3D(disparity, self.Q)
        # Crop by roi_l
        points = points[self.roi_l[1]:self.roi_l[1]+self.roi_l[3], self.roi_l[0]:self.roi_l[0]+self.roi_l[2]]
        # Reshape point3d to array of points
        points = points.reshape(-1, 3)
        # Filter points by min and max z
        mask = (points[:, 2] > min_z) & (points[:, 2] < max_z)
        points = points[mask]
        # Transform points by extrinsic
        points = np.dot(points, self.extrinsic[:3, :3].T) + self.extrinsic[:3, 3]
        if enable_texture:
            if img_raw is not None:
                # texture = cv.cvtColor(img_raw, cv.COLOR_BGR2RGB)
                texture = img_raw
            else:
                texture = cv.cvtColor(img_l, cv.COLOR_GRAY2BGR)
            texture = self.rectifyL(texture)
            texture = texture[self.roi_l[1]:self.roi_l[1]+self.roi_l[3], self.roi_l[0]:self.roi_l[0]+self.roi_l[2]]
            texture = texture.reshape(-1, 3)[mask]/255.0
            return points, texture
        else:
            return points, np.array([])

    def calibPhotometric(self, img_l, img_r, blur_size=31):
        if img_l.shape[2] == 3:
            img_l = cv.cvtColor(img_l, cv.COLOR_BGR2GRAY)
        if img_r.shape[2] == 3:
            img_r = cv.cvtColor(img_r, cv.COLOR_BGR2GRAY)
        img_l = self.undist_l.undist(img_l, self.idx_l)
        img_r = self.undist_r.undist(img_r, self.idx_r)

        # Gaussain blur
        img_l = cv.GaussianBlur(img_l, (blur_size, blur_size), 0)
        img_r = cv.GaussianBlur(img_r, (blur_size, blur_size), 0)

        # Calculate min max
        min_l = np.min(img_l)
        max_l = np.max(img_l)
        min_r = np.min(img_r)
        max_r = np.max(img_r)

        show_img_l = cv.cvtColor(img_l, cv.COLOR_GRAY2BGR)
        show_img_r = cv.cvtColor(img_r, cv.COLOR_GRAY2BGR)

        # print min max on image
        cv.putText(
            show_img_l,
            "min: {}, max: {}".format(min_l, max_l),
            (10, 30),
            cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv.putText(
            show_img_r,
            "min: {}, max: {}".format(min_r, max_r),
            (10, 30),
            cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        # if max=255, then need to puttext 'lower the expousre'
        if max_l == 255:
            cv.putText(
                show_img_l,
                "Please lower the exposure",
                (10, 60), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        if max_r == 255:
            cv.putText(
                show_img_r,
                "Please lower the exposure",
                (10, 60), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        return show_img_l, show_img_r
