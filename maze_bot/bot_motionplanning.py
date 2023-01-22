import cv2
import numpy as np
from math import pow, atan2, sqrt, degrees, asin


class bot_motionplanner():
    def __init__(self):
        self.count = 0
        self.pt_i_taken = False
        self.init_loc = 0
        self.angle_relation_computed = False
        self.bot_angle = 0
        self.bot_angle_rel = 0
        self.bot_angle_s = 0
        self.bot_angle_init = 0

    @staticmethod
    def euler_from_quaternion(x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = atan2(t3, t4)

        return roll, pitch, yaw

    def get_pose(self, data):
        quaternion = data.pose.pose.orientation
        roll, pitch, yaw = self.euler_from_quaternion(
            quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        yaw = degrees(yaw)

        if yaw > 0:
            self.bot_angle_s = yaw

        else:
            self.bot_angle_s = 360 + yaw

    @staticmethod
    def angle_n_distance(pt_a, pt_b):
        error_x = pt_b[0] - pt_a[0]
        error_y = pt_b[1] - pt_a[1]

        distance = sqrt(pow(error_x, 2) + pow(error_y, 2))

        angle = atan2(error_y, error_x)
        angle = degrees(angle)

        if angle > 0:
            return angle, distance
        else:
            return 360 + angle, distance

    def nav_path(self, bot_loc, path, velocity, velocity_publisher):
        if self.count > 20:
            if not self.angle_relation_computed:
                velocity.linear.x = 0.0
                velocity_publisher.publish(velocity)
                self.bot_angle, _ = self.angle_n_distance(
                    self.init_loc, bot_loc)
                self.bot_angle_init = self.bot_angle

                self.bot_angle_rel = self.bot_angle_s - self.bot_angle

                self.angle_relation_computed = True

            else:
                self.bot_angle = self.bot_angle_s - self.bot_angle_rel
                print("\n\nCar angle (Image From Relation) = {} I-S Relation {} Car angle (Simulation) = {}".format(self.bot_angle,self.bot_angle_rel,self.bot_angle_s))
                print("Car angle_Initial (Image) = ",self.bot_angle_init)
                print("Car loc {}".format(bot_loc))

        else:
            if not self.pt_i_taken:
                self.init_loc = bot_loc
                self.pt_i_taken = True

            velocity.linear.x = 1.0
            velocity_publisher.publish(velocity)

            self.count += 1
