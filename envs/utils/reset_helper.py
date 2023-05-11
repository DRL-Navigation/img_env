import random
import math
import numpy as np

from envs.utils import ros_utils

from typing import List
from comn_pkg.msg import Agent as MsgAgent
from comn_pkg.msg import SpeedLimiter
from geometry_msgs.msg import Point
from copy import deepcopy


def get_robot_radius(s1: List, s2: str):
    o = 0
    if s2 == 'circle':
        o = s1[2]
    elif s2 == 'rectangle':
        o = math.sqrt(s1[0] ** 2 + s1[2] ** 2)
    elif s2 == 'leg':
        o = s1[-1] + s1[-2]
    elif s2 == 'L':
        o = math.sqrt(s1[1] ** 2 + s1[3] ** 2)
    elif s2 == 'sweep':
        o = s1[3] + s1[1]  # math.sqrt(s1[1] ** 2 + s1[3] ** 2)
    return o



def random_noise(random_pose):
    random_pose[0] += random.gauss(0, 0.5)
    random_pose[1] += random.gauss(0, 0.5)


def free_check_robo_ped(x, y, poses, d=1.0):
    for i in range(len(poses)):
        pose = poses[i]
        if pose is None:
            continue
        test_d = math.sqrt((x - pose[0]) * (x - pose[0]) + (y - pose[1]) * (y - pose[1]))
        if test_d <= d:
            return False
    return True


def free_check_obj(target_pose, obj_poses):
    for i in range(len(obj_poses)):
        pose = obj_poses[i]
        if pose[-1] == 0.0:
            continue
        d = target_pose[-1] + pose[-1]
        test_d = math.sqrt((target_pose[0] - pose[0]) ** 2 + (target_pose[1] - pose[1]) ** 2)
        if test_d <= d:
            return False
    return True


def _random_pose(x, y, sita):
    return [random.uniform(x[0], x[1]), random.uniform(y[0], y[1]), random.uniform(sita[0], sita[1])]


def random_view(init_pose, pose_range, is_ped=False):
    # task_i = int(epoch / change_task_level_epoch)
    # if task_i > len(view_ranges) - 1:
    #     task_i = len(view_ranges) - 1
    # if not is_ped:
    #     task_view = view_ranges[task_i]
    # else:
    #     task_view = view_ranges_ped[task_i]
    task_view = [2.5, 4.0, 2.5, 4.0]
    rand_pose = None
    while True:
        rand_pose = _random_pose([init_pose[0] - task_view[1], init_pose[0] + task_view[1]],
                                      [init_pose[1] - task_view[3], init_pose[1] + task_view[3]],
                                      [-3.14, 3.14])
        if init_pose[0] - task_view[0] <= rand_pose[0] <= init_pose[0] + task_view[0] and \
                init_pose[1] - task_view[2] <= rand_pose[1] <= init_pose[1] + task_view[2]:
            continue
        if pose_range[0] <= rand_pose[0] <= pose_range[1] and \
                pose_range[2] <= rand_pose[1] <= pose_range[3]:
            break
    return rand_pose


class NearbyPed:
    """
        # ped class which near the robot
    """
    min_dist: List[float] = []

    def __init__(self, robots: int):
        self.min_dist = [float("inf")] * robots

    # TODO try to use __set__
    def set(self, index: int, value: float):
        self.min_dist[index] = value

    def get(self):
        return self.min_dist



#TODO target_dist
class EnvPos:
    
    def __init__(self, cfg):
        self.cfg = cfg

    def init(self):
        robots_msg = self.init_robot()
        peds_msg = self.init_ped()

        return robots_msg, peds_msg

    def reset(self):
        obs_msg = self.reset_obs()
        robots_msg, peds_msg, flag = self._reset_robot_ped()
        while not flag:
            robots_msg, peds_msg, flag = self._reset_robot_ped()
        return obs_msg, robots_msg, peds_msg

    def reset_obs(self):
        self.obs_range = []
        self.obs_object = []
        obs_: List[MsgAgent] = []
        for i in range(self.cfg['object']['total']):
            pose_range = self.cfg['object']['poses'][i]
            size_range = self.cfg['object']['size_range'][i]
            pose_type = self.cfg['object']['poses_type'][i]
            model_shape = self.cfg['object']['shape'][i]

            if model_shape == 'circle':
                model_radius = random.uniform(size_range[0], size_range[1])
            elif model_shape == 'rectangle':
                model_radius = math.sqrt(size_range[0] ** 2 + size_range[2] ** 2)

            if pose_type == 'fix':
                if len(pose_range) == 2:
                    self.obs_range.append(pose_range + [0, model_radius])
                elif len(pose_range) == 3:
                    self.obs_range.append(pose_range + [model_radius])
            elif pose_type == 'range':
                if len(pose_range) == 4:
                    rand_pose = _random_pose(pose_range[:2], pose_range[2:4], [-3.14, 3.14])
                elif len(pose_range) == 6:
                    rand_pose = _random_pose(pose_range[:2], pose_range[2:4], pose_range[4:6])
                self.obs_range.append(rand_pose + [model_radius])
            obs = MsgAgent()
            obs.name = "obstacle"
            obs.ktype = 'obs'
            obs.env_name = self.cfg["env_name"]
            obs.shape = model_shape
            if model_shape == 'circle':
                obs.size = [0, 0, self.obs_range[i][-1]]
            elif model_shape == 'rectangle':
                obs.size = [size_range[0], size_range[1], size_range[2], size_range[3]]
            obs.init_pose.position.x = self.obs_range[i][0]
            obs.init_pose.position.y = self.obs_range[i][1]
            q = ros_utils.rpy_to_q([0, 0, self.obs_range[i][2]])
            obs.init_pose.orientation.x = q[0]
            obs.init_pose.orientation.y = q[1]
            obs.init_pose.orientation.z = q[2]
            obs.init_pose.orientation.w = q[3]
            obs_.append(obs)
        return obs_

    def _get_robo_ped_module_size(self, size, shape):
        list_o = []

        for i in range(len(size)):
            s1 = size[i]
            s2 = shape[i]
            if s2 == 'circle':
                o = s1[2]
            elif s2 == 'rectangle':
                o = math.sqrt(s1[0] ** 2 + s1[2] ** 2)
            elif s2 == 'leg':
                o = s1[-1] + s1[-2]
            elif s2 == 'L':
                o = math.sqrt(s1[1] ** 2 + s1[3] ** 2)
            elif s2 == 'sweep':
                o = s1[3] + s1[1]  # math.sqrt(s1[1] ** 2 + s1[3] ** 2)
            else:
                raise ValueError
            list_o.append(o)
        return list_o


    def _reset_robot_ped(self):
        robo_ped_num = self.cfg['robot']['total'] + self.cfg['ped_sim']['total']
        ped_num = self.cfg['ped_sim']['total']
        robo_num = self.cfg['robot']['total']
        robo_ped_begin_poses_type = self.cfg['robot']['begin_poses_type'][:robo_num] + self.cfg['ped_sim']['begin_poses_type'][:ped_num]
        robo_ped_target_poses_type = self.cfg['robot']['target_poses_type'][:robo_num] + self.cfg['ped_sim']['target_poses_type'][:ped_num]
        robo_ped_begin_poses = self.cfg['robot']['begin_poses'][:robo_num] + self.cfg['ped_sim']['begin_poses'][:ped_num]
        robo_ped_target_poses = self.cfg['robot']['target_poses'][:robo_num] + self.cfg['ped_sim']['target_poses'][:ped_num]
        robo_ped_sizes = self.cfg['robot']['size'][:robo_num] + self.cfg['ped_sim']['size'][:ped_num]
        robo_ped_shape = self.cfg['robot']['shape'][:robo_num] + self.cfg['ped_sim']['shape'][:ped_num]
        robo_ped_module_size = self._get_robo_ped_module_size(robo_ped_sizes, robo_ped_shape)
        self.init_poses = [None] * robo_ped_num
        self.target_poses = [None] * robo_ped_num
        circle_range = random.uniform(self.cfg['circle_ranges'][0], self.cfg['circle_ranges'][1])
        self.circle_range = circle_range
        for i in range(robo_ped_num):
            # 固定起点终点的需要先找出来，其他随机点考虑避开
            if robo_ped_begin_poses_type[i] == 'fix':
                self.init_poses[i] = robo_ped_begin_poses[i]
            if robo_ped_target_poses_type[i] == 'fix':
                self.target_poses[i] = robo_ped_target_poses[i]
            if robo_ped_begin_poses_type[i] == 'rand_angle':
                tmp_pose = robo_ped_begin_poses[i]
                self.init_poses[i] = [tmp_pose[0], tmp_pose[1], random.uniform(tmp_pose[2], tmp_pose[3])]
            if robo_ped_target_poses_type[i] == 'rand_angle':
                tmp_pose = robo_ped_target_poses[i]
                self.target_poses[i] = [tmp_pose[0], tmp_pose[1], random.uniform(tmp_pose[2], tmp_pose[3])]

        circle_ok = False
        while not circle_ok:
            circle_ok = True
            for i in range(robo_ped_num):
                if self.init_poses[i] is not None and self.target_poses[i] is not None:
                    continue  # 不需要随机
                reset_init = True
                while reset_init:
                    goal_fail = 0
                    circle_fail = 0
                    # 先随机起点
                    if 'range' in robo_ped_begin_poses_type[i]:
                        while reset_init:
                            pose_range = robo_ped_begin_poses[i]
                            if 'circle' in robo_ped_begin_poses_type[i]:
                                angle_range = random.uniform(-3.14, 3.14)
                                if 'fix' in robo_ped_begin_poses_type[i]:
                                    angle_range = -3.14 + (6.28 / robo_ped_num) * i
                                rand_pose = [circle_range * math.cos(angle_range) + pose_range[0],
                                             circle_range * math.sin(angle_range) + pose_range[1], angle_range + 3.14]
                                random_noise(rand_pose)
                            else:
                                if 'multi' in robo_ped_begin_poses_type[i]:  # 有多个随机区域可选
                                    pose_range = pose_range[random.randint(0, len(pose_range) - 1)]
                                if len(pose_range) == 4:
                                    rand_pose = _random_pose(pose_range[:2], pose_range[2:4], [-3.14, 3.14])
                                elif len(pose_range) == 6:
                                    rand_pose = _random_pose(pose_range[:2], pose_range[2:4], pose_range[4:6])
                            if free_check_robo_ped(rand_pose[0], rand_pose[1], self.init_poses) and \
                                    free_check_obj([rand_pose[0], rand_pose[1], robo_ped_module_size[i] * 2],
                                                        self.obs_range):
                                self.init_poses[i] = rand_pose[:]
                                reset_init = False
                                break
                            if 'circle' in robo_ped_begin_poses_type[i]:
                                circle_fail += 1
                                if circle_fail > 50:
                                    circle_ok = False
                                    print( "reset robot", i, "fail,number", circle_fail)
                                    for j in range(robo_ped_num):
                                        if 'circle' in robo_ped_begin_poses_type[j]:
                                            self.init_poses[j] = self.target_poses[j] = None
                    # 再随机终点
                    if 'circle_fix' in robo_ped_target_poses_type[i] and self.init_poses[i] is not None:
                        pose_range = robo_ped_target_poses[i]
                        angle = self.init_poses[i][2]
                        self.target_poses[i] = [circle_range * math.cos(angle) + pose_range[0],
                                                circle_range * math.sin(angle) + pose_range[1], angle - 3.14]
                        # self.init_poses[i][2]=random.uniform(-3.14,3.14)
                    if 'range' in robo_ped_target_poses_type[i]:
                        while True:
                            pose_range = robo_ped_target_poses[i]
                            if 'circle' in robo_ped_target_poses_type[i] and self.init_poses[i] is not None:
                                angle = self.init_poses[i][2]
                                rand_pose = [circle_range * math.cos(angle) + pose_range[0],
                                             circle_range * math.sin(angle) + pose_range[1], angle - 3.14]
                                random_noise(rand_pose)
                            if 'multi' in robo_ped_target_poses_type[i]:  # 有多个随机区域可选
                                pose_range = pose_range[random.randint(0, len(pose_range) - 1)]
                            if 'view' in robo_ped_target_poses_type[i]:  # 考虑在起点范围内随机
                                if 'plus' in robo_ped_target_poses_type[i]:
                                    pass
                                    # rand_pose = self.random_view_plus(self.init_poses[i], pose_range)
                                else:
                                    rand_pose = random_view(self.init_poses[i], pose_range)
                            elif len(pose_range) == 4:  # 正常随机
                                rand_pose = _random_pose(pose_range[:2], pose_range[2:4], [-3.14, 3.14])
                            elif len(pose_range) == 6:
                                rand_pose = _random_pose(pose_range[:2], pose_range[2:4], pose_range[4:6])

                            # 依次判断: 不能离起点太近，远离其他目标点， 远离其他起点， 远离已知障碍物 修改：不应该远离其他起点
                            if (self.init_poses[i][0] - rand_pose[0]) ** 2 + (
                                    self.init_poses[i][1] - rand_pose[1]) ** 2 > self.cfg['target_min_dist'] ** 2 \
                                    and free_check_robo_ped(rand_pose[0], rand_pose[1], self.target_poses) and \
                                    free_check_obj([rand_pose[0], rand_pose[1], robo_ped_module_size[i] * 2], self.obs_range,
                                                        ):
                                # 成功找到终点
                                self.target_poses[i] = rand_pose[:]
                                break
                            goal_fail += 1
                            # 多次路径规划失败，考虑重新随机起点
                            if goal_fail > 50:
                                reset_init = True
                                break
        robots_msg, peds_msg = [], []
        flag = True
        for i in range(self.cfg['robot']['total']):
            # print(self.init_poses[i], self.target_poses[i])
            # 根据起点终点初始化机器人
            if self.init_poses[i] is None or self.target_poses[i] is None:
                flag = False
                break
            robot = MsgAgent()
            robot.init_pose.position.x = self.init_poses[i][0]
            robot.init_pose.position.y = self.init_poses[i][1]
            q = ros_utils.rpy_to_q([0, 0, self.init_poses[i][2]])
            robot.init_pose.orientation.x = q[0]
            robot.init_pose.orientation.y = q[1]
            robot.init_pose.orientation.z = q[2]
            robot.init_pose.orientation.w = q[3]
            robot.shape = self.cfg['robot']['shape'][i]
            robot.size = self.cfg['robot']['size'][i]
            robot.goal.x = self.target_poses[i][0]
            robot.goal.y = self.target_poses[i][1]
            robots_msg.append(robot)
        assert self.cfg['ped_sim']['go_back'] in ['yes','no','random']
        for i in range(self.cfg['robot']['total'], robo_ped_num):
            if self.init_poses[i] is None or self.target_poses[i] is None:
                flag = False
                break
            ped = MsgAgent()
            ped.init_pose.position.x = self.init_poses[i][0]
            ped.init_pose.position.y = self.init_poses[i][1]
            q = ros_utils.rpy_to_q([0, 0, self.init_poses[i][2]])
            ped.init_pose.orientation.x = q[0]
            ped.init_pose.orientation.y = q[1]
            ped.init_pose.orientation.z = q[2]
            ped.init_pose.orientation.w = q[3]
            ped.goal.x = self.target_poses[i][0]
            ped.goal.y = self.target_poses[i][1]
            ped.trajectory.append(ped.goal)
            if self.cfg['ped_sim']['go_back'] == 'yes' or (self.cfg['ped_sim']['go_back'] == 'random' and random.random() > 0.5):
                tmp_traj_init_pose = deepcopy(ped.goal)
                tmp_traj_init_pose.x = self.init_poses[i][0]
                tmp_traj_init_pose.y = self.init_poses[i][1]
                ped.trajectory.append(tmp_traj_init_pose)

            peds_msg.append(ped)
        return robots_msg, peds_msg, flag


    def _init_speed_limiter(self):
        speed_limiter_v = SpeedLimiter()
        speed_limiter_w = SpeedLimiter()
        if self.cfg.get("speed_limiter_v"):
            speed_limiter_v.has_velocity_limits = self.cfg['speed_limiter_v'].get("has_velocity_limits", False)
            speed_limiter_v.has_acceleration_limits = self.cfg['speed_limiter_v'].get("has_acceleration_limits", False)
            speed_limiter_v.has_jerk_limits = self.cfg['speed_limiter_v'].get("has_jerk_limits", False)
            speed_limiter_v.min_velocity = self.cfg['speed_limiter_v'].get("min_velocity", 0)
            speed_limiter_v.max_velocity = self.cfg['speed_limiter_v'].get("max_velocity", 0.6)
            speed_limiter_v.min_acceleration = self.cfg['speed_limiter_v'].get("min_acceleration", -2)
            speed_limiter_v.max_acceleration = self.cfg['speed_limiter_v'].get("max_acceleration", 2)
            speed_limiter_v.min_jerk = self.cfg['speed_limiter_v'].get("min_jerk", -2)
            speed_limiter_v.max_jerk = self.cfg['speed_limiter_v'].get("max_jerk", 2)
        if self.cfg.get("speed_limiter_w"):
            speed_limiter_w.has_velocity_limits = self.cfg['speed_limiter_w'].get("has_velocity_limits", False)
            speed_limiter_w.has_acceleration_limits = self.cfg['speed_limiter_w'].get("has_acceleration_limits", False)
            speed_limiter_w.has_jerk_limits = self.cfg['speed_limiter_w'].get("has_jerk_limits", False)
            speed_limiter_w.min_velocity = self.cfg['speed_limiter_w'].get("min_velocity", -0.9)
            speed_limiter_w.max_velocity = self.cfg['speed_limiter_w'].get("max_velocity", 0.9)
            speed_limiter_w.min_acceleration = self.cfg['speed_limiter_w'].get("min_acceleration", -2)
            speed_limiter_w.max_acceleration = self.cfg['speed_limiter_w'].get("max_acceleration", 2)
            speed_limiter_w.min_jerk = self.cfg['speed_limiter_w'].get("min_jerk", -2)
            speed_limiter_w.max_jerk = self.cfg['speed_limiter_w'].get("max_jerk", 2)

        return speed_limiter_v, speed_limiter_w

    def init_robot(self):
        robots_msg = []
        for j in range(self.cfg['robot']['total']):
            robot = MsgAgent()
            robot.ktype = self.cfg['robot_type']
            robot.shape = self.cfg['robot']['shape'][j]
            robot.size = self.cfg['robot']['size'][j]
            robot.name = "cool_robot" + str(j)
            robot.env_name = self.cfg['env_name']
            if self.cfg['robot'].get('sensor_cfgs'):
                robot.sensor_cfg = self.cfg['robot']['sensor_cfgs'][j]
            else:
                robot.sensor_cfg = [0.0, 0.0]
            # assume all robot maintains the same robot control params,
            speed_limiter_v, speed_limiter_w = self._init_speed_limiter()
            robot.speed_limiter_v = speed_limiter_v
            robot.speed_limiter_w = speed_limiter_w

            robots_msg.append(robot)
        return robots_msg

    def init_ped(self):
        peds_msg = []
        for j in range(self.cfg['ped_sim']['total']):
            ped = MsgAgent()
            ped.ktype = self.cfg['ped_sim']['type']  # support: rvo, ervo, pedsim
            if self.cfg['ped_sim']['shape'][j] == 'leg':
                ped.shape = 'leg'
                left_leg = self.cfg['ped_sim']['size'][j]
                right_leg = [left_leg[0], -left_leg[1], left_leg[2]]
                ped.size = left_leg + right_leg
            else:
                ped.shape = self.cfg['ped_sim']['shape'][j]
                ped.size = self.cfg['ped_sim']['size'][j]
            ped.name = "cool_ped" + str(j)
            ped.max_speed = self.cfg['ped_sim']['max_speed'][j]
            ped.env_name = self.cfg['env_name']
            peds_msg.append(ped)
        return peds_msg

    def init_obs(self):
        pass

    @staticmethod
    def init_ped_dataset(peds, ped_pos_v_datas: np.array):
        i = 0
        for ped in peds:
            ped_pos = ped_pos_v_datas[i][:, :3]
            ped_vx_vy = ped_pos_v_datas[i][:, 3:]
            ped.trajectory = [Point(x, y, z) for x,y,z in ped_pos]
            ped.trajectory_v = [Point(x, y, 0) for x,y in ped_vx_vy]
            ped.init_pose.position.x = ped_pos[0][0]
            ped.init_pose.position.y = ped_pos[0][1]
            q = ros_utils.rpy_to_q([0, 0, ped_pos[0][2]])
            ped.init_pose.orientation.x = q[0]
            ped.init_pose.orientation.y = q[1]
            ped.init_pose.orientation.z = q[2]
            ped.init_pose.orientation.w = q[3]

            i += 1
        return

if __name__  == "__main__":
    from env_test import read_yaml
    env_pose = EnvPos(read_yaml('cfg/circle.yaml'))
    env_pose.init()
    env_pose.reset()
