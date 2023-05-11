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
# from comn_pkg.msg import RobotState
from std_msgs.msg import Int8
from collections import deque
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
# from gazebo_msgs.msg import ContactsState
# from gazebo_msgs.msg import ModelState, ModelStates
# from gazebo_msgs.srv import GetModelState
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PointStamped, PoseStamped
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
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


"""
from https://github.com/Daffan/nav-competition-icra2022
"""
def create_model_state(x, y, z, angle, name):
    # the rotation of the angle is in (0, 0, 1) direction
    model_state = ModelState()
    model_state.model_name = name
    model_state.pose.position.x = x
    model_state.pose.position.y = y
    model_state.pose.position.z = z
    model_state.pose.orientation = Quaternion(0, 0, np.sin(angle / 2.), np.cos(angle / 2.))
    model_state.reference_frame = "world"

    return model_state


"""
from  https://github.com/Daffan/nav-competition-icra2022
"""
class GazeboSimulation():

    def __init__(self, init_position=[0, 0, 0], scan_topic='/scan', name="jackal"):
        self._pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self._unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self._reset = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self._model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        self._init_model_state = create_model_state(init_position[0], init_position[1], 0, init_position[2], name)

        self.collision_count = 0
        self._collision_sub = rospy.Subscriber("/collision", Bool, self.collision_monitor)

        self.scan_topic = scan_topic
        self.robot_name = name

    def collision_monitor(self, msg):
        if msg.data:
            self.collision_count += 1

    def get_hard_collision(self):
        # hard collision count since last call
        collided = self.collision_count > 0
        self.collision_count = 0
        return collided

    def pause(self):
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self._pause()
        except rospy.ServiceException:
            print("/gazebo/pause_physics service call failed")

    def unpause(self):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self._unpause()
        except rospy.ServiceException:
            print("/gazebo/unpause_physics service call failed")

    def reset(self):
        """
        /gazebo/reset_world or /gazebo/reset_simulation will
        destroy the world setting, here we used set model state
        to put the model back to the origin
        """
        rospy.wait_for_service("/gazebo/set_model_state")
        try:
            self._reset(self._init_model_state)
            self.collision_count = 0
        except (rospy.ServiceException):
            rospy.logwarn("/gazebo/set_model_state service call failed")

    def get_laser_scan(self):
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message(self.scan_topic, LaserScan, timeout=5)
            except:
                pass
        return data

    def get_model_state(self):
        rospy.wait_for_service("/gazebo/get_model_state")
        try:
            return self._model_state(self.robot_name, 'world')
        except (rospy.ServiceException):
            rospy.logwarn("/gazebo/get_model_state service call failed")

    def reset_init_model_state(self, init_position=[0, 0, 0]):
        """Overwrite the initial model state
        Args:
            init_position (list, optional): initial model state in x, y, z. Defaults to [0, 0, 0].
        """
        self._init_model_state = create_model_state(init_position[0], init_position[1], 0, init_position[2], self.robot_name)


class GazeboEnv(gym.Env):
    def __init__(self, cfg: dict):
        rospy.init_node("drl_gazebo_robot", anonymous=True)

        self._init_datas(cfg)

        self.robot_total = 1
        self.is_collision = 0
        self.is_arrive = False

        # self.last_state_stamp=rospy.Time(secs=0,nsecs=0)
        self.change_task_level_epoch = 10
        self.use_goal_out = False

        self.odom_last = None
        self.state_last = None
        self.target_last = None

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
        # self.state_sub = rospy.Subscriber(self.robot_name + "/state", RobotState, self._state_callback, queue_size=1)
        # self.ped_sub = rospy.Subscriber("/spencer/perception/tracked_persons",
        #                                 TrackedPersons, self._ped_callback, queue_size=1)
        self.global_goal_sub = rospy.Subscriber(self.robot_name + "/global_goal", PoseStamped, self._target_callback,
                                                queue_size=1)
        self.local_goal_sub = rospy.Subscriber(self.robot_name + "/local_goal", PoseStamped, self._local_goal_callback,
                                               queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self._odom_callback, queue_size=1)
        self.vel_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        self.is_done_pub = rospy.Publisher( '/is_done', Int8, queue_size=1)

        self.latest_ped_state, self.latest_ped_image = self._reset_ped_state()
        self.view_ped_time = rospy.get_time()
        self.tf_listener = tf.TransformListener()
        self.tf_target_world = matrix_from_t_q([self.target_global_pose[0], self.target_global_pose[1], 0], (0,0,0,1))

        self.init_os_env()
        self.gazebo_connection = GazeboSimulation(self.start_global_pose, self.scan_topic, self.robot_name)
        print('gazebo env init time', self.view_ped_time)

        self.cur_time = None

    def _init_datas(self, cfg):
        # TODO
        self.robot_radius = get_robot_radius(cfg['robot']['size'][0], cfg['robot']['shape'])
        self.close_distance = cfg.get("robot_radius", 0.13)
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
        self.robot_name = cfg.get("robot_name", 'jackal')
        self.laser_norm = cfg.get('laser_norm', True)

        self.start_global_pose = cfg.get("start_global_pose", (0,0,0))
        self.target_global_pose = cfg.get("target_global_pose", (0,10))
        self.scan_topic = cfg.get("scan_topic", '/scan')
        self.dt = cfg.get('control_hz', 0.4)

    def reset(self, **kwargs):

        # while self.target_last is None:
        #     time.sleep(0.5)
        self.gazebo_connection.reset()
        self.is_collision = 0
        self.is_arrive = False
        print("[reset] get state !", flush=True)
        print('reset', rospy.wait_for_message(self.scan_topic, LaserScan, timeout=5), flush=True)
        state = self.get_state()

        return state

    def _norm_lasers(self, lasers_) -> np.ndarray:
        if self.laser_norm:
            np_lasers = np.array(lasers_) / self.laser_max
            where_are_inf = np.isinf(np_lasers)
            np_lasers[where_are_inf] = 1
        else:
            np_lasers = np.array(lasers_)
            where_are_inf = np.isinf(np_lasers)
            np_lasers[where_are_inf] = self.laser_max
        return np_lasers

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
        is_collisions = [self.is_collision]
        is_arrives = [self.is_arrive]
        lasers = [latest_scan]
        step_ds = [None]
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
                          np.array(nearby_ped),
                          step_ds,
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
        vector_state = state.vector_states[0]
        lasers = state.lasers[0]
        min_dist = self.close_distance
        if self.laser_norm:
            min_dist /= self.laser_max
        print("state", vector_state, flush=True)
        if self.get_goal_dist(vector_state) < 1:
            print("robot arrive !")
            return 5
        elif min(lasers) < min_dist or self.gazebo_connection.get_hard_collision():
            self.is_collision = 1
            print("robot collision !")
            return 1
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
        # rospy.sleep(self.control_hz)
        # print('step once cost', time.time() - self.last_time - self.control_hz, "sleep", self.control_hz)
        # self.last_time = time.time()
        state = self.get_state()
        done = self._is_done(state)
        self.is_done_msg.data = done
        self.is_done_pub.publish(self.is_done_msg)

        return state, np.array([0]), np.array([done>0]), {'dones_info': np.array([done])}

    def robot_control(self, action: ContinuousAction):
        vel = Twist()
        print("action:", action.reverse(), flush=True)
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
        # stamp = self.state_last.laser_image.header.stamp
        scan = self.gazebo_connection.get_laser_scan().ranges
        gazebo_state = self.gazebo_connection.get_model_state()
        vw = [gazebo_state.twist.linear.x, gazebo_state.twist.angular.z]

        min_dist, min_dist_xy = None, None
        # min_dist = self.state_last.min_dist.point.z
        # print("min_dist", min_dist, flush=True)
        # min_dist_xy = [self.state_last.min_dist.point.x, self.state_last.min_dist.point.y]

        # TODO
        # image = self.image_trans(self.state_last.laser_image)
        image = None
        vector_state = self.get_state_goal()
        if self.state_dim == 3:
            pass
        elif self.state_dim == 4:
            vector_state = vector_state[:2] + vw
        elif self.state_dim == 5:
            vector_state = vector_state + vw
        # print(rospy.get_time(), "  ", self.view_ped_time, flush=True)
        if rospy.get_time() - self.view_ped_time > 0.3:
            self.latest_ped_state, self.latest_ped_image = self._reset_ped_state()

        ped_state4sarl = concate_sarl_states(vector_state, self.latest_ped_state, self.robot_radius, self.ped_vec_dim)
        return vector_state, image, min_dist, vw, min_dist_xy, scan, self.latest_ped_state, self.latest_ped_image, ped_state4sarl

    def set_fps(self, fps):
        self.control_hz = 1.0 / fps

    def set_img_size(self, img_size):
        self.image_size = img_size

    def _state_callback(self, msg):
        self.state_last = msg

    def _ped_state(self, msg):
        latest_ped_state, ped_image = self._reset_ped_state()
        j = 0
        for ped in msg.tracks:
            tmx = ped.pose.pose.position.x
            tmy = ped.pose.pose.position.y
            trans, rot = self.tf_listener.lookupTransform('/base_link', '/map', rospy.Time(0.0))
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
        print(self.target_last is None, flush=True)
        self.latest_ped_state, self.latest_ped_image = self._reset_ped_state()
        self.is_collision = False

    def _odom_callback(self, msg):
        self.odom_last = msg

    def init_os_env(self):
        if self.robot_name == 'jackal':
            os.environ["JACKAL_LASER"] = "1"
            os.environ["JACKAL_LASER_MODEL"] = "ust10"
            os.environ["JACKAL_LASER_OFFSET"] = "-0.065 0 0.01"

    def get_state_goal(self):
        pos = self.gazebo_connection.get_model_state().pose.position
        pos_list = [pos.x, pos.y, 0]
        orientation = self.gazebo_connection.get_model_state().pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        tf_base_world = matrix_from_t_q(pos_list, orientation_list)

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
            self.tf_listener.waitForTransform(to_frame, from_frame, timestamp, rospy.Duration(4.0))
            (translation, quaternion) = self.tf_listener.lookupTransform(to_frame, from_frame, timestamp)
            return translation, quaternion
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            rospy.logerr("listen tf from %s to %s error", from_frame, to_frame)