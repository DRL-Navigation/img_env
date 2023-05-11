import gym
import os
import sys

from envs.wrapper import TrajectoryPathHelper


class TestEpisodeWrapper(gym.Wrapper):
    """

    for one robot
    """
    def __init__(self, env, cfg):
        super(TestEpisodeWrapper, self).__init__(env)
        self.cur_episode = 0
        self.max_episodes = cfg["init_pose_bag_episodes"]
        self.dt = cfg['control_hz']
        print("[TestEpisodeWrapper]: Concurrent dt:", self.dt, flush=True)
        self.arrive_num = 0
        self.static_coll_num = 0
        self.ped_coll_num = 0
        self.other_coll_num = 0
        self.steps = 0
        self.tmp_steps = 0
        self.stuck_num = 0
        self.v_sum = 0
        self.w_sum = 0
        self.speed_step = 0

        self.w_variance_array = []
        self.v_jerk_array = []
        self.w_jerk_array = []
        self.w_zero_array = []

        self.traj_helper = TrajectoryPathHelper(dt=self.dt)

    def step(self, action):
        states, reward, done, info = self.env.step(action)
        self.tmp_steps += 1
        speeds = info.get("speeds")[0] # suppose only one agent here
        self.v_sum += speeds[0]
        self.w_sum += abs(speeds[1])

        self.traj_helper.add_vw(*speeds[:2])
        return states, reward, done, info

    def reset(self, **kwargs):
        if self.tmp_steps > 3: # 就2 3步，太短的一般来说有问题
            self.cur_episode += 1
            self.dones_statistics(kwargs.get('dones_info'))

        if self.cur_episode == self.max_episodes:
            self.screen_out()
        self.tmp_steps = 0
        return self.env.reset(**kwargs)

    def dones_statistics(self, t):
        if t is not None:
            t = t[0]
            self.speed_step += self.tmp_steps
            if t == 5:
                self.arrive_num += 1
                self.steps += self.tmp_steps
            elif t == 10:
                self.stuck_num += 1
            elif t == 1:
                self.static_coll_num += 1
            elif t == 2:
                self.ped_coll_num += 1
            elif t == 3:
                self.other_coll_num += 1
            else:
                print("[TestEpisodeWrapper]: No dones info: ", t)
                raise ValueError

            self.path_statistics()

    def path_statistics(self):
        self.traj_helper.reset()
        self.v_jerk_array.append(self.traj_helper.get_v_jerk())
        self.w_jerk_array.append(self.traj_helper.get_w_jerk())
        self.w_zero_array.append(self.traj_helper.get_w_zero())
        self.w_variance_array.append(self.traj_helper.get_w_variance())

        self.traj_helper.clear_vw_array()

    def screen_out(self):
        print("""\n
                    ###############################
                    [TestEpisodeWrapper]: Have run max episodes {}, statistics number are in the following:

                    arrive_rate: {},
                    static_coll_rate: {},
                    ped_coll_rate: {},
                    other_coll_rate: {},
                    avg_arrive_steps: {},
                    stuck_rate: {},
                    avg_v: {},
                    avg_w: {},
                    avg_w_variance: {},
                    avg_v_jerk: {},
                    avg_w_jerk: {},
                    avg_w_zero: {}, 
                    """.format(self.max_episodes,
                               self.arrive_num / self.max_episodes,
                               self.static_coll_num / self.max_episodes,
                               self.ped_coll_num / self.max_episodes,
                               self.other_coll_num / self.max_episodes,
                               self.steps / max(1, self.arrive_num),
                               self.stuck_num / self.max_episodes,
                               self.v_sum / self.speed_step,
                               self.w_sum / self.speed_step,
                               sum(self.w_variance_array) / self.max_episodes,
                               sum(self.v_jerk_array) / self.max_episodes,
                               sum(self.w_jerk_array) / self.max_episodes,
                               sum(self.w_zero_array) / self.max_episodes,
                               ), flush=True)
        print("[TestEpisodeWrapper]: Exit Progress!")
        sys.exit()