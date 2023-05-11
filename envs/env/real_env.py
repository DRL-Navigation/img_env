import rospy
import gym
import math
import random
import os
import numpy as np
import time
import cv2
import copy
import tf

# from gz_ros import *
from comn_pkg.msg import RobotState
from std_msgs.msg import Int8
from collections import deque
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
# from gazebo_msgs.msg import ContactsState
# from gazebo_msgs.msg import ModelState, ModelStates
# from gazebo_msgs.srv import GetModelState
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PointStamped, PoseStamped

try:
    from spencer_tracking_msgs.msg import TrackedPersons, TrackedPerson
except:
    print("No installion about spencer_tracking_msgs")
from typing import List, Tuple

from envs.utils import q_to_rpy, matrix_from_t_q, transform_point, inverse, t_from_matrix, rpy_from_matrix, mul_matrix
from envs.utils import _trans_lidar_log_map
from envs.state import ImageState
from envs.action import ContinuousAction
from envs.utils import concate_sarl_states
from envs.utils import get_robot_radius


def distance(x1, y1, x2, y2):
    return (x1 - x2) ** 2 + (y1 - y2) ** 2


class RealEnv(gym.Env):
    def __init__(self, cfg: dict):
        rospy.init_node("drl_real_robot")
        self._init_datas(cfg)

        self.robot_total = 1
        self.is_collision = False
        self.is_arrive = False

        # self.last_state_stamp=rospy.Time(secs=0,nsecs=0)
        self.change_task_level_epoch = 10
        self.use_goal_out = False

        self.odom_last = None
        self.scan_img_state_last = None
        self.target_last = None
        self.tf_target_world = None

        self.arrival_dist = 0.5
        self.collision_th = 0.05
        self.is_done_msg = Int8()

        # self.discrete_actions = \
        #     [[0.0, -0.9], [0.0, -0.6], [0.0, -0.3], [0.0, 0.05], [0.0, 0.3], [0.0, 0.6], [0.0, 0.9],
        #      [0.2, -0.9], [0.2, -0.6], [0.2, -0.3], [0.2, 0], [0.2, 0.3], [0.2, 0.6], [0.2, 0.9],
        #      [0.4, -0.9], [0.4, -0.6], [0.4, -0.3], [0.4, 0], [0.4, 0.3], [0.4, 0.6], [0.4, 0.9],
        #      [0.6, -0.9], [0.6, -0.6], [0.6, -0.3], [0.6, 0], [0.6, 0.3], [0.6, 0.6], [0.6, 0.9]]

        self.bridge = CvBridge()

        self.last_time = time.time()
        self.tf_listener = tf.TransformListener()
        self.robot_name = ''
        self.state_sub = rospy.Subscriber(self.robot_name + "/state", RobotState, self._state_callback, queue_size=1)
        self.ped_sub = rospy.Subscriber("/spencer/perception/tracked_persons",
                                        TrackedPersons, self._ped_callback, queue_size=1)
        self.global_goal_sub = rospy.Subscriber(self.robot_name + "/global_goal", PoseStamped, self._target_callback,
                                                queue_size=1)
        self.local_goal_sub = rospy.Subscriber(self.robot_name + "/local_goal", PoseStamped, self._local_goal_callback,
                                               queue_size=1)
        self.odom_sub = rospy.Subscriber(self.robot_name + '/odom', Odometry, self._odom_callback, queue_size=1)
        self.vel_pub = rospy.Publisher(self.robot_name + self.cmd_topic, Twist, queue_size=1)
        self.is_done_pub = rospy.Publisher(self.robot_name + '/is_done', Int8, queue_size=1)
        self.scan_sub = rospy.Subscriber(self.robot_name + "/scan", LaserScan, self._scan_callback, queue_size=1)
        self.latest_ped_state, self.latest_ped_image = self._reset_ped_state()
        self.view_ped_time = rospy.get_time()
        print('real env init time', self.view_ped_time)
        self.cur_time = None

    def _init_datas(self, cfg):
        # TODO
        self.robot_radius = get_robot_radius(cfg['robot']['size'][0], cfg['robot']['shape'])
        self.map_file = cfg['global_map']['map_file']
        self.global_resolution = cfg['global_map']['resolution']
        self.view_map_resolution = cfg['view_map']['resolution']
        self.view_map_size = (cfg['view_map']['width'], cfg['view_map']['height'])
        self.robot_total = cfg['robot']['total']  # * self.env_num
        self.ped_total = cfg['ped_sim']['total']  # * self.env_num
        self.max_ped = cfg['max_ped']
        self.ped_vec_dim = cfg['ped_vec_dim']
        self.ped_image_r_base = cfg['ped_image_r']
        self.image_size = tuple(cfg['image_size'])
        self.ped_image_size = tuple(cfg['ped_image_size'])
        self.ped_map_resolution = 6.0 / self.ped_image_size[0]
        self.ped_image_r = self.ped_image_r_base
        self.control_hz = cfg['control_hz']
        self.state_dim = cfg['state_dim']
        self.laser_max = cfg['laser_max']
        self.cmd_topic = cfg.get('cmd_topic', '/cmd_vel')
        self.laser_norm = cfg.get('laser_norm', True)
        self.dt = cfg.get('control_hz', 0.4)

        self.base_link_frame = cfg.get("base_link_frame", "base_link")
        self.world_frame = cfg.get("world_frame", "map")
        self.laser_frame = cfg.get("laser_frame", "laser_link")

        print("[RealEnv] cfg set -> base_link_frame: {}, world_frame: {}", self.base_link_frame, self.world_frame,
              flush=True)
        self.a = [1]
        self.tmp_distances = None

    def reset(self, **kwargs):
        while self.tf_target_world is None:
            print("waiting for target goal ...", flush=True)
            time.sleep(0.5)
        print("get state !", flush=True)
        state = self.get_state()
        return state

    def get_state(self) -> ImageState:
        if self.is_collision:
            raise Exception
        state, image_last, min_dist, vw, min_dist_xy, latest_scan, ped_state, ped_image, ped_state4sarl = self._get_robot_state()
        vec_states, sensor_maps, lasers, ped_infos, ped_maps, is_collisions, is_arrives, distances = [], [], [], [], [], [], [], []
        vec_states = [state]
        static_maps = [None]
        sensor_maps = [image_last]
        # print(latest_scan, flush=True)
        # sensor_maps = [_trans_lidar_log_map(latest_scan)]
        is_collisions = [False]
        is_arrives = [self.get_goal_dist(state) < self.arrival_dist]
        lasers = [self._deal_scan(latest_scan)]
        # if  len(lasers[0]) == 361:
        #     lasers[0].pop(0)

        distances = math.sqrt(state[0] ** 2 + state[1] ** 2)
        step_ds = (self.tmp_distances - distances) if (self.tmp_distances is not None) else 0
        self.tmp_distances = distances

        step_ds = [step_ds]
        tmp_ktypes = [0]
        nearby_ped = [None]
        ped_infos = [ped_state]
        ped_maps = [ped_image]
        ped_states4sarl = [ped_state4sarl]
        raw_sensor_maps = [None]
        return ImageState(np.array(vec_states),
                          # np.array(static_maps),
                          np.array(sensor_maps),
                          np.array(is_collisions),
                          np.array(is_arrives),
                          self._norm_lasers(lasers),
                          np.array(ped_infos),
                          np.array(ped_maps),
                          step_ds,
                          np.array(nearby_ped),
                          # np.array(tmp_ktypes, dtype=np.uint8),
                          # np.array(ped_states4sarl),
                          # np.array(raw_sensor_maps)
                          )

    def get_goal_dist(self, state):
        x = state[0]
        y = state[1]
        dist = math.sqrt(x * x + y * y)
        print('distance cur and goal:', dist)
        return dist

    def _is_done(self, state):
        if state.is_arrives[0]:
            self.is_arrive = True
            self.target_last = None
            print("robot arrive !")
            return 1
        elif state.is_collisions[0]:
            self.is_collision = True
            print("robot collision !")
            return 2
        return 0

    def time_control(self):
        ############## Time Controller ###############
        while self.cur_time is not None and time.time() - self.cur_time < self.dt:
            time.sleep(0.01)
        if self.cur_time:
            print("once step time:", time.time() - self.cur_time, flush=True)
        self.cur_time = time.time()

    def step(self, actions: List[ContinuousAction]):
        self.time_control()
        self.robot_control(actions[0])

        state = self.get_state()
        done = self._is_done(state)
        self.is_done_msg.data = done
        self.is_done_pub.publish(self.is_done_msg)

        return state, np.array([0]), np.array([done]), {'dones_info': np.zeros_like([done])}

    def robot_control(self, action: ContinuousAction):
        vel = Twist()
        print("action:", action.reverse())
        vel.linear.x = action.v
        vel.angular.z = action.w
        self.vel_pub.publish(vel)
        # self.beep(action[1])

    def beep(self, b):
        if b == 1:
            # spd-say "BBBBBBBBBBBB"  spd-say 让系统读出指定的语句
            os.system("bash ~/beep")

    def image_trans(self, img_ros):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_ros, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)
        image = cv2.resize(cv_image, self.image_size) / 255.0
        return image

    def _get_robot_state(self):
        # stamp = self.scan_img_state_last.laser_image.header.stamp
        scan = self.scan_last
        vw = [self.odom_last.twist.twist.linear.x, self.odom_last.twist.twist.angular.z]
        min_dist = None  # self.scan_img_state_last.min_dist.point.z
        # print("min_diust",min_dist, flush=True)
        min_dist_xy = None  # [self.scan_img_state_last.min_dist.point.x, self.scan_img_state_last.min_dist.point.y]
        image = self.image_trans(self.scan_img_state_last.laser_image) if self.scan_img_state_last else None
        # vector_state = [self.scan_img_state_last.pose.position.x, self.scan_img_state_last.pose.position.y]
        # goal_rpy = q_to_rpy([self.scan_img_state_last.pose.orientation.x, self.scan_img_state_last.pose.orientation.y,
        #                           self.scan_img_state_last.pose.orientation.z, self.scan_img_state_last.pose.orientation.w])

        vector_state = self.get_state_goal()
        if self.state_dim == 3:
            pass
        elif self.state_dim == 4:
            vector_state = vector_state[:2] + vw
        elif self.state_dim == 5:
            vector_state = vector_state + vw

        print(rospy.get_time(), "  ", self.view_ped_time, flush=True)
        if rospy.get_time() - self.view_ped_time > 0.3:
            self.latest_ped_state, self.latest_ped_image = self._reset_ped_state()

        ped_state4sarl = concate_sarl_states(vector_state, self.latest_ped_state, self.robot_radius, self.ped_vec_dim)
        return vector_state, image, min_dist, vw, min_dist_xy, scan, self.latest_ped_state, self.latest_ped_image, ped_state4sarl

    def set_fps(self, fps):
        self.control_hz = 1.0 / fps

    def set_img_size(self, img_size):
        self.image_size = img_size

    def _state_callback(self, msg):
        self.scan_img_state_last = msg

    def _ped_state(self, msg):
        latest_ped_state, ped_image = self._reset_ped_state()
        j = 0
        for ped in msg.tracks:
            tmx = ped.pose.pose.position.x
            tmy = ped.pose.pose.position.y
            trans, rot = self.tf_listener.lookupTransform(self.base_link_frame, self.world_frame, rospy.Time(0.0))
            m = matrix_from_t_q(trans, rot)
            tmx, tmy, _ = transform_point(m, [tmx, tmy, ped.pose.pose.position.z])
            # print("+++++++++++++++")
            #    print('tmx, tmy', tmx, tmy)
            vx, vy = ped.twist.twist.linear.x, ped.twist.twist.linear.y
            latest_ped_state[j * self.ped_vec_dim + 1] = tmx
            latest_ped_state[j * self.ped_vec_dim + 2] = tmy
            latest_ped_state[j * self.ped_vec_dim + 3] = vx
            latest_ped_state[j * self.ped_vec_dim + 4] = vy
            latest_ped_state[j * self.ped_vec_dim + 5] = self.ped_image_r * 2  # leg radius
            latest_ped_state[j * self.ped_vec_dim + 6] = self.ped_image_r * 2 + self.robot_radius
            latest_ped_state[j * self.ped_vec_dim + 7] = math.sqrt(tmx ** 2 + tmy ** 2)
            if tmx > 3 or tmx < -3 or tmy > 3 or tmy < -3:
                continue
            tmx, tmy = -tmx + 3, -tmy + 3

            # below this need change in future.
            # draw grid which midpoint in circle
            coor_tmx = (tmx - self.ped_image_r) // self.ped_map_resolution, (
                        tmx + self.ped_image_r) // self.ped_map_resolution
            coor_tmy = (tmy - self.ped_image_r) // self.ped_map_resolution, (
                        tmy + self.ped_image_r) // self.ped_map_resolution
            coor_tmx = list(map(int, coor_tmx))
            coor_tmy = list(map(int, coor_tmy))

            for jj in range(*coor_tmx):
                for kk in range(*coor_tmy):
                    if jj < 0 or jj >= self.ped_image_size[0] or kk < 0 or kk >= self.ped_image_size[1]:
                        pass
                    else:
                        if distance((jj + 0.5) * self.ped_map_resolution, (kk + 0.5) * self.ped_map_resolution, tmx,
                                    tmy) < self.ped_image_r ** 2:
                            # Put 3 values per pixel
                            ped_image[:, jj, kk] = 1.0, vx, vy
            # imageio.imsave("{}ped.png".format(self.index), ped_image[0])
            j += 1
        return latest_ped_state, ped_image

    def _reset_ped_state(self):
        latest_ped_state = np.zeros([self.max_ped * self.ped_vec_dim + 1], dtype=np.float32)
        ped_image = np.zeros([3, self.ped_image_size[0], self.ped_image_size[1]], dtype=np.float32)
        return latest_ped_state, ped_image

    def _norm_lasers(self, lasers_) -> np.ndarray:
        if self.laser_norm:
            np_lasers = np.array(lasers_) / self.laser_max
            where_are_inf = np.isinf(np_lasers)
            np_lasers[where_are_inf] = 1
            where_are_nan = np.isnan(np_lasers)
            np_lasers[where_are_nan] = 1
        else:
            np_lasers = np.array(lasers_)
            where_are_inf = np.isinf(np_lasers)
            np_lasers[where_are_inf] = self.laser_max
            np_lasers = np.clip(np.array(lasers_), 0, self.laser_max)
            where_are_nan = np.isnan(np_lasers)
            np_lasers[where_are_nan] = self.laser_max
        return np_lasers

    def get_state_goal(self):
        tf_base_world = self.get_tf_matrix(self.base_link_frame, self.world_frame, rospy.Time.now())
        tf_target_base = mul_matrix(inverse(tf_base_world), self.tf_target_world)
        t = t_from_matrix(tf_target_base)
        rpy = rpy_from_matrix(tf_target_base)
        state = [t[0], t[1], rpy[2]]
        return state

    def get_tf_matrix(self, from_frame, to_frame, timestamp):
        t, q = self.get_tf(from_frame, to_frame, timestamp)
        return matrix_from_t_q(t, q)

    def get_tf(self, from_frame, to_frame, timestamp):
        try:
            self.tf_listener.waitForTransform(to_frame, from_frame, rospy.Time(), rospy.Duration(4))
            (translation, quaternion) = self.tf_listener.lookupTransform(to_frame, from_frame, rospy.Time(0.0))
            return translation, quaternion
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            rospy.logerr("listen tf from %s to %s error", from_frame, to_frame)

    def _ped_callback(self, msg):
        self.latest_ped_state, self.latest_ped_image = self._ped_state(msg)
        # self.view_ped = copy.deepcopy(self.latest_ped_image)
        self.view_ped_time = int(msg.header.stamp.secs) + float("0." + str(msg.header.stamp.nsecs))
        # print('get ped time', self.view_ped_time)

    def _target_callback(self, msg):
        self.is_collision = False

    def _local_goal_callback(self, msg):
        print("get local goal !!", flush=True)
        self.target_last = msg
        self.latest_ped_state, self.latest_ped_image = self._reset_ped_state()

        pose = [msg.pose.position.x, msg.pose.position.y]
        t, q = self.get_tf('base_link', 'map', rospy.Time.now())
        self.tf_target_world = matrix_from_t_q([pose[0], pose[1], 0], q)
        self.is_collision = False

    def _odom_callback(self, msg):
        self.odom_last = msg

    def _scan_callback(self, msg):
        self.scan_last = msg

    def _deal_scan(self, laser):
        if laser.header.frame_id == self.base_link_frame:
            return laser.ranges
        s_t = time.time()
        angle_min = laser.angle_min
        angle_increment = laser.angle_increment
        cur_angle = angle_min
        transformed_ranges = []

        trans, rot = self.tf_listener.lookupTransform(self.base_link_frame, self.laser_frame, rospy.Time(0.0))
        m = matrix_from_t_q(trans, rot)

        for x in laser.ranges:
            point = (x*math.cos(cur_angle), x*math.sin(cur_angle), 0)
            xyz = transform_point(m, point)
            # print(point, xyz, flush=True)
            transformed_ranges.append(math.sqrt(xyz[0] ** 2 + xyz[1] ** 2))
            cur_angle += angle_increment
        # print("dealing scan time: ",  time.time() - s_t, flush=True)
        return transformed_ranges