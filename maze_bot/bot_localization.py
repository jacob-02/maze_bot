import cv2
import numpy as np
from maze_bot.utilities import ret_smallest_obj


class bot_localizer():

    def __init__(self):
        self.is_bg_captured = False
        self.bg_model = []
        self.maze_og = []
        self.orig_X = 0
        self.orig_Y = 0
        self.orig_rows = 0
        self.orig_cols = 0
        self.tranform_arr = []
        self.orig_rot = 0
        self.rot_mat = 0

    @staticmethod
    def ret_rois_boundinghull(rois_mask, cnts):
        maze_enclosure = np.zeros_like(rois_mask)

        if cnts:
            cnts_ = np.concatenate(cnts)
            cnts_ = np.array(cnts_)
            cv2.fillConvexPoly(maze_enclosure, cnts_, 255)

        cnts_largest = cv2.findContours(
            maze_enclosure, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]
        hull = cv2.convexHull(cnts_largest[0])
        cv2.drawContours(maze_enclosure, [hull], 0, 255, 2)
        return hull

    def update_frameofrefrence_parameters(self, X, Y, W, H, rot_angle):
        self.orig_X = X
        self.orig_Y = Y
        self.orig_rows = H
        self.orig_cols = W
        self.orig_rot = rot_angle  # 90 degree counterClockwise
        self.transform_arr = [X, Y, W, H]
        # Rotation Matrix
        self.rot_mat = np.array(
            [
                [np.cos(np.deg2rad(self.orig_rot)),
                 np.sin(np.deg2rad(self.orig_rot))],
                [-np.sin(np.deg2rad(self.orig_rot)),
                 np.cos(np.deg2rad(self.orig_rot))]
            ]
        )
        self.rot_mat_rev = np.array(
            [
                [np.cos(np.deg2rad(-self.orig_rot)),
                 np.sin(np.deg2rad(-self.orig_rot))],
                [-np.sin(np.deg2rad(-self.orig_rot)),
                 np.cos(np.deg2rad(-self.orig_rot))]
            ]
        )

    def extract_bg(self, curr_frame):
        gray_frame = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray_frame, 50, 150, gray_frame)
        cnts = cv2.findContours(edges, cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[0]

        rois_mask = np.zeros(
            (curr_frame.shape[0], curr_frame.shape[1]), dtype=np.uint8)

        for idx, _ in enumerate(cnts):
            cv2.drawContours(rois_mask, cnts, idx, 255, -1)

        min_cntr_idx = ret_smallest_obj(cnts)
        rois_noCar_mask = rois_mask.copy()

        if min_cntr_idx != -1:
            cv2.drawContours(rois_noCar_mask, cnts, min_cntr_idx, 0, -1)

            car_mask = np.zeros_like(rois_mask)
            cv2.drawContours(car_mask, cnts, min_cntr_idx, 255, -1)
            cv2.drawContours(car_mask, cnts, min_cntr_idx, 255, 3)
            notcar_mask = cv2.bitwise_not(car_mask)
            frame_car_remvd = cv2.bitwise_and(
                curr_frame, curr_frame, mask=notcar_mask)

            base_clr = frame_car_remvd[0][0]
            grnd_replica = np.ones_like(curr_frame)*base_clr

            self.bg_model = cv2.bitwise_and(
                grnd_replica, grnd_replica, mask=rois_noCar_mask)
            self.bg_model = cv2.bitwise_or(self.bg_model, frame_car_remvd)

        hull = self.ret_rois_boundinghull(rois_mask, cnts)
        [x, y, w, h] = cv2.boundingRect(hull)

        maze = rois_noCar_mask[y:y+h, x:x+w]
        maze_occupencygrid = cv2.bitwise_not(maze)

        self.maze_og = cv2.rotate(maze_occupencygrid, cv2.ROTATE_90_CLOCKWISE)
        self.update_frameofrefrence_parameters(x, y, w, h, 90)

        cv2.imshow("bg_model", self.bg_model)
        cv2.imshow("maze_og", self.maze_og)
        cv2.imshow("rois_mask", rois_mask)
        cv2.waitKey(0)
        cv2.destroyAllWindows()



    def localize_bot(self, curr_frame, frame_disp):
        if not self.is_bg_captured:
            self.extract_bg(curr_frame)
            self.is_bg_captured = True
    


