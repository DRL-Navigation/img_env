import gym
import os
import sys

from envs.wrapper import TrajectoryPathHelper


class RealTestRecoderWrapper(gym.Wrapper):
    def __init__(self, env, cfg):
        super(RealTestRecoderWrapper, self).__init__(env)
        self.cfg = cfg
        self.traj_helper = TrajectoryPathHelper(dt=cfg['control_hz'])
        if not os.path.exists("../output"):
            os.mkdir("../output")
        self.output_file = cfg.get("output_file", "../output/barndata_output_ppo.txt")

    def step(self, action):
        states, reward, done, info = self.env.step(action)
        speeds = info.get("speeds")[0] # suppose only one agent here

        self.traj_helper.add_vw(*speeds[:2])
        self.out2logfile(info.get('dones_info'))
        return states, reward, done, info

    def out2logfile(self, dones):
        if dones is None or dones==0:
            return

        dones = dones[0]
        self.traj_helper.reset()
        f = open(self.output_file, "a")
        metric_dict = self.traj_helper.get_metric_dict()
        metric_dict['arrive'] = 1 if dones == 5 else 0 # arrive
        metric_dict['static_collision'] = 1 if dones == 1 else 0 # static collision
        metric_dict['stuck'] = 1 if dones == 10 else 0 # time out
        f.write("{arrive}, {static_collision}, {stuck}, "
                "{v_avg}, {w_avg}, {v_acc}, {w_acc}, {v_jerk}, {w_jerk}, {w_zero}, {path_time}, {steps}".format_map(metric_dict))
        f.write("\n")
        f.flush()
        f.close()

        print("Running Over!", flush=True)
        self.traj_helper.clear_vw_array()
        sys.exit()
