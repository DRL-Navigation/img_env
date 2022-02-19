# -*- coding: utf-8 -*-
import cv2
import numpy as np
import copy
import os.path as osp
import rospkg
import rospy
import gym
import math
import time
import rosbag
import imageio

from cv_bridge import CvBridge
from rospy.service import ServiceException
from typing import Tuple, List
from copy import deepcopy

from envs.utils import EnvPos, NearbyPed
from envs.state import ImageState
from envs.action import Action, ContinuousAction
from comn_pkg.srv import InitEnv, InitEnvRequest, ResetEnv, ResetEnvRequest, StepEnv, StepEnvRequest, EndEp, EndEpRequest
from comn_pkg.msg import AgentState, EnvsInfo, EnvReset, EnvInit
from comn_pkg.msg import Env as MsgEnv
import os.path as osp
import os


# TODO change 7 and 10 from config


def get_pkg_path(pkg_name):
    rospack = rospkg.RosPack()
    return rospack.get_path(pkg_name)


def get_map_file(file_name):
    pkg_path = get_pkg_path('img_env')
    final_file = osp.abspath(osp.join(pkg_path, '../../../', 'envs', 'map', file_name))
    # print(final_file, flush=True)
    return final_file


def distance(x1, y1, x2, y2):
    return (x1 - x2) ** 2 + (y1 - y2) ** 2


class ImageEnv(gym.Env):
    test: bool

    env_name: str
    # env_num: int
    epoch: int = 0
    # envs: List[ImageEnv]

    robot_total: int = 0
    ped_total: int = 0

    node_id: str = '0'

    init_req: InitEnvRequest

    control_hz: float
    env_type: str
    robot_type: str
    robot_radius: float
    ped_leg_radius: float
    laser_max: float
    ped_safety_space: float

    image_batch: int
    state_batch: int
    state_dim: int
    state_normalize: bool
    laser_batch: int
    act_dim: int

    resolution: float
    global_resolution: float
    view_map_resolution: float
    view_map_size: Tuple[float, float]

    image_size: Tuple
    ped_image_size: Tuple
    circle_ranges: Tuple

    max_ped: int
    ped_vec_dim: int
    ped_image_r: float

    actions: Action
    dones: np.ndarray

    episode_envs_info: EnvsInfo = None

    map_file: str
    yaml_file: str
    cfg_name: str
    cfg_type: str

    cfg: dict

    nearby_ped: NearbyPed
    state: ImageState
    env_pose: EnvPos

    def __init__(self, cfg: dict):
        self.cfg = cfg
        self._init_static_param(cfg)

        self.env_pose = EnvPos(cfg)
        self.nearby_ped = NearbyPed(self.robot_total)
        self.bridge = CvBridge()
        if self.node_id == '0':  # only start once in one python script
            rospy.init_node("{}_pynode".format(self.env_name))

        self.init_req = InitEnvRequest()
        self.step_req = StepEnvRequest()
        self.reset_req = ResetEnvRequest()

        # self.change_task_level_epoch = 10
        # self.muti_env_shif = [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]]
        if self.cfg_type == 'bag':
            self.bag_repeat = 20
            self.reset_reqs = []
            self.reset_index = 0
            self.load_envs_bag(cfg['init_pose_bag_name'])
        self.init_env()

    def _init_static_param(self, cfg):
        """
            static param
        """
        self.test = cfg['test']
        self.node_id = cfg['env_id']
        assert self.node_id < cfg['env_num']

        self.env_name = cfg['env_name']
        self.env_type = cfg['env_type']
        self.robot_type = cfg['robot_type']
        # TODO cfg type
        self.cfg_name = cfg['cfg_name']
        self.cfg_type = cfg['cfg_type']

        self.image_batch = cfg['image_batch']
        self.image_size = tuple(cfg['image_size'])
        self.ped_image_size = tuple(cfg['ped_image_size'])
        self.state_batch = cfg['state_batch']
        self.state_dim = cfg['state_dim']
        self.state_normalize = cfg['state_normalize']
        self.laser_batch = cfg['laser_batch']
        self.laser_max = cfg['laser_max']
        self.act_dim = cfg['act_dim']
        self.circle_ranges = tuple(cfg['circle_ranges'])
        self.ped_safety_space = cfg['ped_safety_space']
        self.ped_leg_radius = cfg['ped_leg_radius']
        self.robot_radius = cfg['robot_radius']
        self.control_hz = cfg['control_hz']

        self.map_file = cfg['global_map']['map_file']
        self.resolution = 6.0 / self.ped_image_size[0]
        self.global_resolution = cfg['global_map']['resolution']
        self.view_map_resolution = cfg['view_map']['resolution']
        self.view_map_size = (cfg['view_map']['width'], cfg['view_map']['height'])
        self.robot_total = cfg['robot']['total'] #* self.env_num
        self.ped_total = cfg['ped_sim']['total'] #* self.env_num
        self.max_ped = cfg['max_ped']
        self.ped_vec_dim = cfg['ped_vec_dim']
        self.ped_image_r = cfg['ped_image_r']

        self.node_id = str(cfg['node_id'])
        cpp_node_name = "{}{}".format(self.env_name, self.node_id)
        self.init_env_service_name = cpp_node_name + '/init_image_env' #+ self.node_id
        self.reset_env_service_name = cpp_node_name + '/reset_image_env' #+ self.node_id
        self.step_env_service_name = cpp_node_name + '/step_image_env'# + self.node_id
        self.end_ep_service_name = cpp_node_name + '/ep_end_image_env'# + self.node_id

    def _init_req(self):
        self.init_req.view_resolution = self.view_map_resolution
        self.init_req.view_width, self.init_req.view_height = self.view_map_size
        self.init_req.step_hz = self.control_hz
        self.init_req.state_dim = self.state_dim
        self.init_req.is_show_gui = self.cfg['show_gui']
        self.init_req.sleep_t = self.cfg['sleep_t']
        self.init_req.window_height = self.cfg['window_height']
        self.init_req.show_image_height = self.cfg['show_image_height']
        self.init_req.is_draw_step = self.cfg['is_draw_step']
        self.init_req.step_draw = self.cfg['step_draw']
        self.init_req.use_laser = self.cfg['use_laser']
        self.init_req.range_total = self.cfg['range_total']
        self.init_req.view_angle_begin = self.cfg['view_angle_begin']
        self.init_req.view_angle_end = self.cfg['view_angle_end']
        self.init_req.view_min_dist = self.cfg['view_min_dist']
        self.init_req.view_max_dist = self.cfg['view_max_dist']
        self.init_req.relation_ped_robo = self.cfg['relation_ped_robo']

        env = MsgEnv()
        env.map_file = get_map_file(self.map_file)
        env.name = self.env_name
        env.global_resolution = self.global_resolution

        env.robots, env.peds = self.env_pose.init()
        env.ped_scene_type = env.peds[0].ktype if len(env.peds) > 0 else ""
        self.init_req.env = copy.deepcopy(env)

    def init_env(self):
        self._init_req()
        try:
            rospy.wait_for_service(self.init_env_service_name)
            init_env_srv = rospy.ServiceProxy(self.init_env_service_name, InitEnv)
            init_env_srv(self.init_req)
            return True
        except ServiceException as e:
            print (str(e))
            print ( "init env service error" )
            return False

    def _reset_req(self):
        self.beep_times_ = [0] * self.robot_total
        self.tmp_distances = None
        # // TODO 多态
        if self.cfg_type == 'yaml':
            self.step_req.is_test = self.reset_req.is_test = self.test
            self.step_req.env_id = self.reset_req.env_id = int(self.node_id)
            self.reset_req.obstacles, self.reset_req.robots, self.reset_req.peds = self.env_pose.reset()

            self.step_req.robots = self.reset_req.robots
        elif self.cfg_type == 'bag':
            if self.reset_index < len(self.reset_reqs) * self.bag_repeat:
                self.step_req.is_test = self.reset_req.is_test = self.test
                self.step_req.env_id = self.reset_req.env_id = int(self.node_id)
                rq = self.reset_reqs[self.reset_index % len(self.reset_reqs)]
                self.reset_req.obstacles, self.reset_req.robots, self.reset_req.peds = rq.obstacles, rq.robots, rq.peds

                self.step_req.robots = self.reset_req.robots

                self.reset_index += 1
            # self.reset_req.robots = self.pyrobots.reset(self.cfg)
            # self.reset_req.peds = self.pypeds.reset(self.cfg)
        # elif self.cfg_type == 'bag':
        #     if self.reset_index < len(self.reset_reqs) * self.bag_repeat:
        #         self.init_poses = []
        #         self.target_poses = []
        #         self.reset_req = self.reset_reqs[self.reset_index % len(self.reset_reqs)]
        #         self.reset_index += 1
        #         for i in range(self.robot_total):
        #             init_pose = [self.reset_req.robots[i].init_pose.position.x,
        #                          self.reset_req.robots[i].init_pose.position.y]
        #             target_pose = [self.reset_req.robots[i].goal.x, self.reset_req.robots[i].goal.y]
        #             self.init_poses.append(init_pose)
        #             self.target_poses.append(target_pose)
        #     else:
        #         print("bag env finish!")
        #         return False

    def save_envs_bag(self, ep_total, bag_name):
        pkg_path = get_pkg_path('img_env')
        output_dir = osp.abspath(osp.join(pkg_path, 'cfg_bag/'))
        if not osp.exists(output_dir):
            os.makedirs(output_dir)
        bag_file = osp.join(output_dir, bag_name)
        print(bag_file)
        bag = rosbag.Bag(bag_file, 'w')
        try:
            init_env_msg = EnvInit()
            init_env_msg.view_resolution = self.init_req.view_resolution
            init_env_msg.view_width = self.init_req.view_width
            init_env_msg.view_height = self.init_req.view_height
            init_env_msg.step_hz = self.init_req.step_hz
            init_env_msg.state_dim = self.init_req.state_dim
            init_env_msg.env = self.init_req.env
            bag.write('init_env', init_env_msg)
            for i in range(ep_total):
                self.reset()
                reset_env_msg = EnvReset()
                reset_env_msg.peds = self.reset_req.peds
                reset_env_msg.obstacles = self.reset_req.obstacles
                reset_env_msg.robots = self.reset_req.robots
                reset_env_msg.is_test = True
                bag.write('reset_env', reset_env_msg)
        finally:
            bag.close()

    def reset(self, fall=0, **kwargs):

        self._reset_req()
        # call ros reset service
        try:
            rospy.wait_for_service(self.reset_env_service_name)
            reset_env_srv = rospy.ServiceProxy(self.reset_env_service_name, ResetEnv)
            reset_res = reset_env_srv(self.reset_req)
        except ServiceException as e:
            if fall < 10:
                time.sleep(2)
                print(str(e), "try to retry again, fall time", fall)
                return self.reset(fall + 1)
            else:
                print("reset env service error, max fall time", fall)
                raise ValueError

        state = self._get_states(reset_res.robot_states)
        # 1 arrive, -1 coll, 0 otherwise.
        rewards = np.zeros_like(state.is_collisions)
        self.dones = np.zeros_like(state.is_collisions)
        return state

    def _step_req(self, actions: List[ContinuousAction]):
        for i in range(self.robot_total):
            action: ContinuousAction = actions[i]
            if self.dones[i] == 0:
                self.step_req.robots[i].alive = True
                self.step_req.robots[i].v = action.v
                self.step_req.robots[i].w = action.w
                self.step_req.robots[i].v_y = action.beep
            else:
                self.step_req.robots[i].alive = False
                self.step_req.robots[i].v = 0
                self.step_req.robots[i].w = 0
                self.step_req.robots[i].v_y = 0

    def load_envs_bag(self, bag_name):
        pkg_path = get_pkg_path('img_env')
        output_dir = osp.abspath(osp.join(pkg_path, 'cfg_bag/'))
        bag_file = osp.join(output_dir, bag_name)
        bag = rosbag.Bag(bag_file)
        for topic, msg, t in bag.read_messages(topics=['init_env', 'reset_env']):
            # if topic == 'init_env':
            #     self.init_req.view_resolution = msg.view_resolution
            #     self.init_req.view_width = msg.view_width
            #     self.init_req.view_height = msg.view_height
            #     self.init_req.step_hz = msg.step_hz
            #     self.init_req.state_dim = msg.state_dim
            #     self.init_req.env = msg.env
            #     self.robot_total=0

            #     env = msg.env
            if topic == 'reset_env':
                reset_req = ResetEnvRequest()
                reset_req.obstacles = msg.obstacles
                reset_req.robots = msg.robots
                reset_req.peds = msg.peds
                reset_req.is_test = msg.is_test
                self.reset_reqs.append(reset_req)

        bag.close()

    def step(self, actions: List[ContinuousAction]):

        self._step_req(actions)
        try:
            rospy.wait_for_service(self.step_env_service_name)
            step_env_srv = rospy.ServiceProxy(self.step_env_service_name, StepEnv)
            step_res = step_env_srv(self.step_req)
        except ServiceException as e:
            print(str(e))
            print("step env service error")
            return False
        state = self._get_states(step_res.robot_states)
        # 1 arrive, -1 coll, 0 otherwise.
        rewards = state.is_arrives - state.is_collisions
        self.dones = np.clip(state.is_collisions, -1, 1) + state.is_arrives
        # sometimes a robot will be in reach and collision at the same time, so dones may be more than 1. need clip.
        self.dones = np.clip(self.dones, 0, 1)
        return state, rewards, deepcopy(self.dones), {'dones_info': np.zeros_like(self.dones)}

    def end_ep(self, robot_res):
        try:
            request = EndEpRequest()
            request.robot_res = robot_res[:]
            rospy.wait_for_service(self.end_ep_service_name)
            end_ep_srv = rospy.ServiceProxy(self.end_ep_service_name, EndEp)
            end_ep_srv(request)
            return True
        except ServiceException as e:
            print(e.message)
            print("end ep service error")
            return False

    def _draw_ped_map(self, ped_tmp: np.ndarray, robot_state: AgentState, robot_index: int) -> np.ndarray:
        """
            draw the pedestrian map, which consisted of 3 channels [X veloicty, Y velocity, pos]
            detail information, see paper:
        """
        ped_tmp[0] = len(robot_state.pedinfo)
        ped_image = np.zeros([3, self.ped_image_size[0], self.ped_image_size[1]], dtype=np.float32)
        for j in range(int(ped_tmp[0])):
            rt = robot_state.pedinfo[j]
            ped_tmp[j * self.ped_vec_dim + 1] = rt.px
            ped_tmp[j * self.ped_vec_dim + 2] = rt.py
            ped_tmp[j * self.ped_vec_dim + 3] = rt.vx
            ped_tmp[j * self.ped_vec_dim + 4] = rt.vy
            ped_r = round(rt.r_, 2)
            ped_tmp[j * self.ped_vec_dim + 5] = ped_r
            ped_tmp[j * self.ped_vec_dim + 6] = ped_r + self.init_req.env.robots[robot_index].size[-1]
            ped_tmp[j * self.ped_vec_dim + 7] = math.sqrt(rt.px ** 2 + rt.py ** 2)
            if rt.px > 3 or rt.px < -3 or rt.py > 3 or rt.py < -3:
                continue
            tmx, tmy = -rt.px + 3, -rt.py + 3

            # below this need change in future.
            # draw grid which midpoint in circle
            coor_tmx = (tmx - self.ped_image_r) // self.resolution, (tmx + self.ped_image_r) // self.resolution
            coor_tmy = (tmy - self.ped_image_r) // self.resolution, (tmy + self.ped_image_r) // self.resolution
            coor_tmx = list(map(int, coor_tmx))
            coor_tmy = list(map(int, coor_tmy))

            for jj in range(*coor_tmx):
                for kk in range(*coor_tmy):
                    if jj < 0 or jj >= self.ped_image_size[0] or kk < 0 or kk >= self.ped_image_size[1]:
                        pass
                    else:
                        if distance((jj + 0.5) * self.resolution, (kk + 0.5) * self.resolution, tmx,
                                    tmy) < self.ped_image_r ** 2:
                            # Put 3 values per pixel
                            ped_image[:, jj, kk] = 1.0, rt.vx, rt.vy
        # imageio.imsave("{}ped.png".format(self.index), ped_image[0])
        return ped_image

    def _trans_cv2_sensor_map(self, view_map):
        cv_image = self.bridge.imgmsg_to_cv2(view_map, desired_encoding='8UC1')
        img_data = cv2.resize(cv_image.astype('uint8'), (self.image_size[0], self.image_size[1]),
                              interpolation=cv2.INTER_CUBIC)
        # imageio.imsave("{}.png".format(self.index), img_data)
        # self.index += 1

        return img_data.astype('float16') / 255.0

    def _get_states(self, robot_states):
        vec_states, sensor_maps, lasers, ped_infos, ped_maps, is_collisions, is_arrives, distances = [], [], [], [], [], [], [], []

        for i in range(self.robot_total):
            robot_state = robot_states[i]
            robot_state.pedinfo.sort(key=lambda x: x.px ** 2 + x.py ** 2)
            ped_tmp = np.zeros([self.max_ped * self.ped_vec_dim + 1], dtype=np.float32)
            ped_image = self._draw_ped_map(ped_tmp, robot_state, i)

            if len(robot_state.pedinfo) != 0:
                self.nearby_ped.set(i, ped_tmp[7] - ped_tmp[6])  # since we have sorted pedinfo before

            ped_infos.append(ped_tmp[:])
            ped_maps.append(copy.deepcopy(ped_image))
            is_collisions.append(robot_state.is_collision)
            is_arrives.append(robot_state.is_arrive)
            vec_states.append(robot_state.state)
            # print('arrive ', robot_state.is_arrive)
            sensor_maps.append(self._trans_cv2_sensor_map(robot_state.view_map))
            lasers.append(robot_state.laser)

            distances.append(math.sqrt(robot_state.state[0] ** 2 + robot_state.state[1] ** 2))
        step_ds = self.tmp_distances - np.array(distances) if self.tmp_distances is not None else np.zeros_like(
            distances)
        # print("step_ds: ", step_ds)
        self.tmp_distances = np.array(distances)
        return ImageState(np.array(vec_states),
                          np.array(sensor_maps),
                          np.array(is_collisions),
                          np.array(is_arrives),
                          np.array(lasers) / self.laser_max,
                          np.array(ped_infos),
                          np.array(ped_maps),
                          step_ds,
                          self.nearby_ped.get()
                          )

# TODO vector env 一个结束了 另一个还在跑，结束的那个要马上reset。 即 reset 和 step 返回的数据应该要一致。
# TODO 解决方案？ reset 和 step 的 service 为单env 服务？ 缺点： 阻塞了，之后多线程跑不方便，

# TODO 写好文档，reward 怎么包起来。
# TODO 还有必要单独整一个env吗？ 除了reset obs 这些函数，其他都不需要。
# TODO 最外面套一层ROS通信类，

# TODO 有一些robot撞了之后，动作可以继续前向，但是送给网络的数据要过滤掉。
# TODO resolution readme 解释

# TODO reset lasers 也返回数据

