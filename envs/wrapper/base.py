import logging
import gym
import numpy as np
import math
import yaml
import time

from typing import *
from collections import deque
from copy import deepcopy


from envs.state import ImageState
from envs.action import *


class StatePedVectorWrapper(gym.ObservationWrapper):
    avg = np.array([0.0, 0.0, 0.0, 0.0, 0.25, 0.25, 0.0])
    std = np.array([6.0, 6.0, 0.6, 0.9, 0.50, 0.5, 6.0])

    def __init__(self, env, cfg=None):
        super(StatePedVectorWrapper, self).__init__(env)

    def observation(self, state: ImageState):
        self._normalize_ped_state(state.ped_vector_states)
        return state

    def _normalize_ped_state(self, peds):

        for robot_i_peds in peds:
            for j in range(int(robot_i_peds[0])): # j: ped index
                robot_i_peds[1 + j * 7:1 + (j + 1) * 7] = (robot_i_peds[1 + j * 7:1 + (j + 1) * 7] - self.avg) / self.std


class VelActionWrapper(gym.Wrapper):
    def __init__(self, env, cfg):
        super(VelActionWrapper, self).__init__(env)
        if cfg['discrete_action']:
            self.actions: DiscreteActions = DiscreteActions(cfg['discrete_actions'])

            self.f = lambda x: self.actions[int(x)] if np.isscalar(x) else ContinuousAction(*x)
        else:
            self.f = lambda x: ContinuousAction(*x)

    def step(self, action):
        action = self.action(action)
        state, reward, done, info = self.env.step(action)
        info['speeds'] = np.array([a.reverse()[:2] for a in action])
        return state, reward, done, info

    def action(self, actions) -> Iterator[ContinuousAction]:
        return list(map(self.f, actions))

    def reverse_action(self, actions):

        return actions


class MultiRobotCleanWrapper(gym.Wrapper):
    is_clean : list
    def __init__(self, env, cfg):
        super(MultiRobotCleanWrapper, self).__init__(env)
        self.is_clean = np.array([True] * cfg['agent_num_per_env'])

    def step(self, action):
        state, reward, done, info = self.env.step(action)
        info['is_clean'] = deepcopy(self.is_clean)
        reward[~info['is_clean']] = 0
        info['speeds'][~info['is_clean']] = np.zeros(2)
        # for i in range(len(done)):
        #     if done[i]:
        #         self.is_clean[i]=False
        self.is_clean = np.where(done>0, False, self.is_clean)
        return state, reward, done, info

    def reset(self, **kwargs):
        state = self.env.reset(**kwargs)
        self.is_clean = np.array([True] * len(self.is_clean))
        return state



class StateBatchWrapper(gym.Wrapper):
    batch_dict: np.ndarray

    def __init__(self, env, cfg):
        print(cfg,flush=True)
        super(StateBatchWrapper, self).__init__(env)
        self.q_sensor_maps = deque([], maxlen=cfg['image_batch']) if cfg['image_batch']>0 else None
        self.q_vector_states = deque([], maxlen=cfg['state_batch']) if cfg['state_batch']>0 else None
        self.q_lasers = deque([], maxlen=cfg['laser_batch']) if cfg['laser_batch']>0 else None
        self.batch_dict = {
            "sensor_maps": self.q_sensor_maps,
            "vector_states": self.q_vector_states,
            "lasers": self.q_lasers,
        }

    def step(self, action):
        state, reward, done, info = self.env.step(action)
        return self.batch_state(state), reward, done, info

    def _concate(self, b: str, t: np.ndarray):
        q = self.batch_dict[b]
        if q is None:
            return t
        else:
            t = np.expand_dims(t, axis=1)
        # start situation
        while len(q) < q.maxlen:
            q.append(np.zeros_like(t))
        q.append(t)
        #  [n(Robot), k(batch), 84, 84]
        return np.concatenate(list(q), axis=1)

    def batch_state(self, state):
        # TODO transpose. print
        state.sensor_maps = self._concate("sensor_maps", state.sensor_maps)
        # print('sensor_maps shape; ', state.sensor_maps.shape)

        # [n(robot), k(batch), state_dim] -> [n(robot), k(batch) * state_dim]
        tmp_ = self._concate("vector_states", state.vector_states)
        state.vector_states = tmp_.reshape(tmp_.shape[0], tmp_.shape[1] * tmp_.shape[2])
        # print("vector_states shape", state.vector_states.shape)


        state.lasers = self._concate("lasers", state.lasers)
        # print("lasers shape:", state.lasers.shape)
        return state

    def reset(self, **kwargs):
        state = self.env.reset(**kwargs)
        return self.batch_state(state)


class SensorsPaperRewardWrapper(gym.Wrapper):
    def __init__(self, env, cfg):
        super(SensorsPaperRewardWrapper, self).__init__(env)

        self.ped_safety_space = cfg['ped_safety_space']
        self.cfg = cfg

    def step(self, action):
        states, reward, done, info = self.env.step(action)
        return states, self.reward(reward, states), done, info

    def _each_r(self, states: ImageState, index: int):
        distance_reward_factor = 200
        collision_reward = reach_reward = step_reward = distance_reward = rotation_reward = beep_reward = 0

        min_dist = states.ped_min_dists[index]
        vector_state = states.vector_states[index]
        is_collision = states.is_collisions[index]
        is_arrive = states.is_arrives[index]
        step_d = states.step_ds[index]

        if min_dist <= self.ped_safety_space:
            collision_reward = -50 * (self.ped_safety_space - min_dist)
        if is_collision > 0:
            collision_reward = -500
        else:
            d = math.sqrt(vector_state[0] ** 2 + vector_state[1] ** 2)
            # print 'robot ',i," dist to goal: ", d
            if d < 0.3 or is_arrive:
                reach_reward = 500.0
            else:
                distance_reward = step_d * distance_reward_factor
                step_reward = -5

        reward = collision_reward + reach_reward + step_reward + distance_reward + beep_reward
        return reward

    def reward(self, reward, states):
        rewards = np.zeros(len(states))
        for i in range(len(states)):
            rewards[i] = self._each_r(states, i)

        return rewards


class NeverStopWrapper(gym.Wrapper):
    """
        NOTE !!!!!!!!!!!
        put this in last wrapper.
    """
    def __init__(self, env, cfg):
        super(NeverStopWrapper, self).__init__(env)

    def step(self, action):
        states, reward, done, info = self.env.step(action)
        if info['all_down'][0]:
            states = self.env.reset(**info)

        return states, reward, done, info


# time limit
class TimeLimitWrapper(gym.Wrapper):
    def __init__(self, env, cfg):
        super(TimeLimitWrapper, self).__init__(env)
        self._max_episode_steps = cfg['time_max']
        robot_total = cfg['robot']['total']
        self._elapsed_steps = np.zeros(robot_total, dtype=np.uint8)

    def step(self, ac):
        observation, reward, done, info = self.env.step(ac)
        self._elapsed_steps += 1
        done = np.where(self._elapsed_steps > self._max_episode_steps, 1, done)
        info['dones_info'] = np.where(self._elapsed_steps > self._max_episode_steps, 10, info['dones_info'])
        return observation, reward, done, info

    def reset(self, **kwargs):
        self._elapsed_steps = 0
        return self.env.reset(**kwargs)###


class InfoLogWrapper(gym.Wrapper):
    def __init__(self, env, cfg):
        super(InfoLogWrapper, self).__init__(env)
        robot_total = cfg['robot']['total']
        self.tmp = np.zeros(robot_total, dtype=np.uint8)

    def step(self, action):
        states, reward, done, info = self.env.step(action)
        info['arrive'] = states.is_arrives
        info['collision'] = states.is_collisions

        info['dones_info'] = np.where(states.is_collisions > 0, states.is_collisions, info['dones_info'])
        info['dones_info'] = np.where(states.is_arrives == 1, 5, info['dones_info'])
        info['all_down'] = self.tmp + sum(np.where(done>0, 1, 0)) == len(done)
        return states, reward, done, info




class TestEpisodeWrapper(gym.Wrapper):
    """

    for one robot
    """
    def __init__(self, env, cfg):
        super(TestEpisodeWrapper, self).__init__(env)
        self.cur_episode = -1
        self.max_episodes = cfg["init_pose_bag_episodes"]
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

    def step(self, action):
        states, reward, done, info = self.env.step(action)
        self.tmp_steps += 1
        speeds = info.get("speeds")[0] # suppose only one agent here
        self.v_sum += speeds[0]
        self.w_sum += abs(speeds[1])
        return states, reward, done, info

    def reset(self, **kwargs):
        self.cur_episode += 1
        t = kwargs.get('dones_info')
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
        self.tmp_steps = 0
        if self.cur_episode == self.max_episodes:
            print("""
            arrive_rate: {},
            static_coll_rate: {},
            ped_coll_rate: {},
            other_coll_rate: {},
            avg_arrive_steps: {},
            stuck_rate: {},
            avg_v: {},
            avg_w:{},
            """.format(self.arrive_num / self.max_episodes,
                       self.static_coll_num / self.max_episodes,
                       self.ped_coll_num / self.max_episodes,
                       self.other_coll_num / self.max_episodes,
                       self.steps / max(1, self.arrive_num),
                       self.stuck_num / self.max_episodes,
                       self.v_sum / self.speed_step,
                       self.w_sum / self.speed_step,
                       ), flush=True)

        return self.env.reset(**kwargs)


