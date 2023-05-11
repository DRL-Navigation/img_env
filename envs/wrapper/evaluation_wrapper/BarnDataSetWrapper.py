import gym
import os
import rospkg
import sys
import time
import subprocess

from envs.wrapper import TrajectoryPathHelper

from os.path import join
from typing import Callable, List, Optional, Tuple


class BarnDataSetWrapper(gym.Wrapper):
    """

    for one robot
    """
    def __init__(self, env, cfg):
        super(BarnDataSetWrapper, self).__init__(env)
        self.gazebo_process = None
        self.cur_world = 0
        self.show_gui = cfg.get('show_gui', True)

        rospack = rospkg.RosPack()
        self.base_path = rospack.get_path('gazebo_env')
        self.launch_file = join(self.base_path, 'launch', cfg['world_base_file'])

        self.repeated_time_per_env = cfg.get("repeated_time_per_env", 1)
        self.cur_repeated_time_per_env = 0
        self.dt = cfg['control_hz']
        self.max_worlds = 300
        self.traj_helper = TrajectoryPathHelper(dt=self.dt)

        self.output_file = cfg.get("output_file", "../output/barndata_output_ppo.txt")
        self.output_dir = '/'.join(self.output_file.split("/")[:-1])
        if self.output_dir != '' and not os.path.exists(self.output_dir):
            os.mkdir(self.output_dir)

        self.start = True

    def step(self, action):
        states, reward, done, info = self.env.step(action)
        speeds = info.get("speeds")[0] # suppose only one agent here

        self.traj_helper.add_vw(*speeds[:2])
        return states, reward, done, info

    def reset(self, **kwargs):
        self.out2logfile(kwargs.get('dones_info'))

        print("BarnDataSetWrapper Reset", flush=True)
        if self.cur_world == self.max_worlds:
            print("[BarnDataSetWrapper]: Run Over.", flush=True)
            self.gazebo_process.terminate()
            sys.exit()

        # self.cur_repeated_time_per_env += 1
        self.change_world()

        state = self.env.reset(**kwargs)

        # if self.cur_repeated_time_per_env == self.repeated_time_per_env:
        #     # self.change_world()
        #     self.cur_world += 1
        #     self.cur_repeated_time_per_env *= 0

        return state

    def change_world(self):
        # gazebo 换图
        if self.gazebo_process is not None:
            self.gazebo_process.terminate()
            os.system("ps -ef|grep gazebo|grep -v grep | grep -v gazebo_cfg |awk '{print $2}'|xargs kill -9")
            # time.sleep(20)
        print("[BarnDataSetWrapper]: cur_world", self.cur_world, flush=True)

        self.gazebo_process = subprocess.Popen([
            'roslaunch',
            self.launch_file,
            'world_name:=' + join(self.base_path, "worlds", "BARN", "world_{}.world".format(str(self.cur_world))),
            'gui:=' + ("true" if self.show_gui else "false")
        ])

        time.sleep(5)

    def out2logfile(self, dones):
        if dones is None:
            return
        self.cur_repeated_time_per_env += 1

        dones = dones[0]
        self.traj_helper.reset()
        f = open(self.output_file, "a")
        metric_dict = self.traj_helper.get_metric_dict()
        metric_dict['arrive'] = 1 if dones == 5 else 0 # arrive
        metric_dict['static_collision'] = 1 if dones == 1 else 0 # static collision
        metric_dict['stuck'] = 1 if dones == 10 else 0 # time out
        metric_dict['cur_world'] = self.cur_world
        f.write("{cur_world}, {arrive}, {static_collision}, {stuck}, "
                "{v_avg}, {w_avg}, {v_acc}, {w_acc}, {v_jerk}, {w_jerk}, {w_zero}, {path_time}, {steps}".format_map(metric_dict))
        f.write("\n")
        f.close()

        self.traj_helper.clear_vw_array()

        if self.cur_repeated_time_per_env == self.repeated_time_per_env:
            # self.change_world()
            self.cur_world += 1
            self.cur_repeated_time_per_env *= 0