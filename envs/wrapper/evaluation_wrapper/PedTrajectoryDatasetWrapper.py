import gym
import numpy as np
import pandas as pd
import os
import sys
import scipy

# sys.path.append("/home/ustc/qiuqc/drlnav_master/drlnav_frame/USTC_lab/env/drlnav_env/")
# print(sys.path)
from envs.wrapper import TrajectoryPathHelper

from typing import Callable, List, Optional, Tuple


class PedTrajectoryDatasetWrapper(gym.Wrapper):
    def __init__(self, env, cfg):
        super(PedTrajectoryDatasetWrapper, self).__init__(env)
        self.cfg = cfg
        self.dt = cfg.get('control_hz', 0.4)
        assert cfg['ped_traj_dataset'] is not None
        self._read_dataset(cfg['ped_traj_dataset'])

        self.repeated_time_per_env = cfg.get("repeated_time_per_env", 10)
        self.cur_repeated_time_per_env = 0
        self.ped_dataset_worlds = cfg.get('ped_dataset_worlds', [[0,10]])
        self.max_worlds = len(self.ped_dataset_worlds)
        self.cur_world = 0
        cfg['ped_sim']['total'] = self.cur_world_max_peds = self.ped_dataset_worlds[self.cur_world][1] - self.ped_dataset_worlds[self.cur_world][0] + 1

        self.traj_helper = TrajectoryPathHelper(dt=self.dt)
        self.node_id = cfg.get("node_id", 0)
        self.output_file = cfg.get("output_file", "../output/ped_dataset_{}/ppo_{}.txt".format(self.dataset_name, self.node_id))
        self.output_dir = '/'.join(self.output_file.split("/")[:-1])
        if self.output_dir != '' and not os.path.exists(self.output_dir):
            os.mkdir(self.output_dir)

    def step(self, action):
        states, reward, done, info = self.env.step(action)
        speeds = info.get("speeds")[0] # suppose only one agent here

        self.traj_helper.add_vw(*speeds[:2])
        return states, reward, done, info

    def reset(self, **kwargs):
        self.out2logfile(kwargs.get('dones_info'))

        print("PedTrajectoryDatasetWrapper Reset ", flush=True)
        if self.cur_world == self.max_worlds:
            print("[PedTrajectoryDatasetWrapper]: Run Over.", flush=True)
            sys.exit()

        # self.cur_repeated_time_per_env += 1
        kwargs['cur_ped_pos_v_datas'] = self.change_world()
        state = self.env.reset(**kwargs)

        # if self.cur_repeated_time_per_env == self.repeated_time_per_env:
        #     # self.change_world()
        #     self.cur_world += 1
        #     self.cur_repeated_time_per_env *= 0

        return state

    def out2logfile(self, dones):
        if dones is None:
            return
        self.cur_repeated_time_per_env += 1

        dones = dones[0]
        self.traj_helper.reset()
        f = open(self.output_file, "a")
        metric_dict = self.traj_helper.get_metric_dict()
        metric_dict['arrive'] = 1 if dones == 5 else 0 # arrive
        metric_dict['ped_collision'] = 1 if dones == 2 else 0 # ped collision
        metric_dict['stuck'] = 1 if dones == 10 else 0 # time out
        metric_dict['cur_world'] = self.cur_world
        f.write("{cur_world}, {arrive}, {ped_collision}, {stuck}, "
                "{v_avg}, {w_avg}, {v_acc}, {w_acc}, {v_jerk}, {w_jerk}, {w_zero}, {path_time}, {steps}".format_map(metric_dict))
        f.write("\n")
        f.close()

        self.traj_helper.clear_vw_array()

        if self.cur_repeated_time_per_env == self.repeated_time_per_env:
            # self.change_world()
            self.cur_world += 1
            self.cur_repeated_time_per_env *= 0

    def _read_dataset(self, path: str):
        if not os.path.exists(path):
            # run in drlnav_frame
            path = sys.argv[0].split("runner")[0] + "env/drlnav_env/" + path
        world_df: pd.DataFrame = pd.read_csv(path, header=None).T
        world_df.columns = ["frame", "ped", "y", "x"]
        world_df[["frame", "ped"]] = world_df[["frame", "ped"]].astype("int")
        all_peds: np.ndarray = np.unique(world_df.ped)
        self.max_peds: int = max(all_peds)

        self.dataset_name = self.cfg.get("ped_dataset_name", 'eth')
        self.world_df = world_df
        self.swapxy = self.cfg.get("swapxy", True)
        self.spawn_delay_s = self.cfg.get("spawn_delay_s", 0)
        self.offset = self.cfg.get("offset", [1.4, 14.4, 0])
        self.fps = self.cfg.get("fps", 15)
        # fps 表示每15个frame代表1s， 和原数据集2.5fps对应（间隔6frame一个数据）
        assert (self.dt * self.fps) % 1 == 0
        self.skip_frame = int(self.dt * self.fps)
        self.start_t = self.cfg.get("start_t", 0)
        self.max_time = self.cfg.get("max_time", 20)
        self.scale_x = self.cfg.get("scale_x", 1)
        self.scale_y = self.cfg.get("scale_y", 1)

    def _generate_humans(self, start_idx, max_agents) -> List[List[float]]:
        ped_pos_v_datas = []
        max_time = 0
        for i in range(max_agents):
            ped_id: int = i + start_idx + 1
            # "PedTrajectoryDatasetWrapper: ERROR! ped_id should small than max_peds"
            assert ped_id < self.max_peds + 1
            ped_i = self.world_df[self.world_df.ped == ped_id]
            # gather data
            if i == 0:
                # update start frame to be representative of "first" pedestrian
                start_frame = list(ped_i["frame"])[0]
            cur_frame = list(ped_i["frame"])[0]
            t_data = PrerecordedHuman.gather_times(
                ped_i, self.spawn_delay_s, self.start_t, start_frame, self.fps
            )
            if (ped_i.frame.iloc[0] - start_frame) / self.fps > self.max_time:
                # assuming the data of the agents is sorted relatively based off time
                break
            print(
                'cur_node_id %s, cur_world %s, cur_repeat_time %s, Generating recorded humans from "%s" in range [%d, %d]: %d\r'
                % (self.node_id, self.cur_world, self.cur_repeated_time_per_env, self.dataset_name, start_idx, start_idx+max_agents, ped_id),
                end="",
            )
            xytheta_data = PrerecordedHuman.gather_posn_data(
                ped_i, self.offset, swap_axes=self.swapxy, scale_x=self.scale_x, scale_y=self.scale_y
            )
            interp_fns = PrerecordedHuman.init_interp_fns(xytheta_data, t_data)
            pos_v_data = PrerecordedHuman.gather_vel_data(t_data, xytheta_data)
            assert len(xytheta_data) == len(pos_v_data)
            pos_v_data = [pos_v_data[0]] * (cur_frame-start_frame) + pos_v_data
            pos_v_data = pos_v_data[::self.skip_frame]
            max_time = max(max_time, len(pos_v_data))
            ped_pos_v_datas.append(pos_v_data)
            # print(pos_v_data)
        for i in range(max_agents):
            ped_pos_v_datas[i] = ped_pos_v_datas[i] + (max_time - len(ped_pos_v_datas[i])) * [ped_pos_v_datas[i][-1]]
        return ped_pos_v_datas

    def change_world(self):

        ped_pos_v_datas = self._generate_humans(self.ped_dataset_worlds[self.cur_world][0],
                                                self.cur_world_max_peds)

        return ped_pos_v_datas


def euclidean_dist2(p1: List[float], p2: List[float]) -> float:
    """Compute the 2D euclidean distance from p1 to p2.

    Args:
        p1 (list): A point in a 2D space (with at least 2 dimens).
        p2 (list): Another point in a 2D space (with at least 2 dimens).

    Returns:
        dist (float): the euclidean (straight-line) distance between the points.
    """
    diff_x: float = p1[0] - p2[0]
    diff_y: float = p1[1] - p2[1]
    return np.sqrt(diff_x ** 2 + diff_y ** 2)


class PrerecordedHuman:
    """ BEGIN INITIALIZATION UTILS """
    @staticmethod
    def init_interp_fns(
        posn_data: List[List[float]], times: List[float]
    ) -> Tuple[
        Callable[[float], float], Callable[[float], float], Callable[[float], float]
    ]:
        posn_data = np.array(posn_data)
        ts = np.array(times)
        # correct for the fact that times of 0 is weird
        ts[0] = ts[1] - (ts[2] - ts[1])

        x = posn_data[:, 0]
        y = posn_data[:, 1]
        th = posn_data[:, 2]
        interp = scipy.interpolate.interp1d
        xfunc = interp(ts, x, bounds_error=False, fill_value=(x[0], x[-1]))
        yfunc = interp(ts, y, bounds_error=False, fill_value=(y[0], y[-1]))
        thfunc = interp(ts, th, bounds_error=False, fill_value=(th[0], th[-1]))
        return xfunc, yfunc, thfunc

    @staticmethod
    def gather_times(
        ped_i: int, time_delay: float, start_t: float, start_frame: int, fps: float
    ) -> List[float]:
        times = (ped_i["frame"] - start_frame) * (1.0 / fps)
        # account for the time delay (before the rest of the action),
        # and the start time (when the pedestrian first appears in the simulator)
        times += time_delay + start_t
        # convert pd df column to list
        times = list(times)
        # add the first time step (after spawning, before moving)
        times = [times[0] - start_t] + times
        return times

    @staticmethod
    def gather_posn_data(
        ped_i: int,
        offset: Tuple[int, int, int],
        swap_axes: Optional[bool] = False,
        scale_x: Optional[int] = 1,
        scale_y: Optional[int] = 1,
    ) -> np.ndarray:
        xy_data = []
        xy_order = ("x", "y")
        scale = (scale_x, scale_y)
        if swap_axes:
            xy_order = ("y", "x")
            scale = (scale_y, scale_x)
        # gather the data from df
        xy_data = np.array(
            [scale[0] * ped_i[xy_order[0]], scale[1] * ped_i[xy_order[1]]]
        )
        # apply the rotations to the x, y positions
        s = np.sin(offset[2])
        c = np.cos(offset[2])
        # construct xy data
        posn_data = np.array(
            [
                xy_data[0] * c - xy_data[1] * s + offset[0],
                xy_data[0] * s + xy_data[1] * c + offset[1],
            ]
        )
        # append vector angles for all the agents
        now = posn_data[:, 1:]  # skip first
        last = posn_data[:, :-1]  # skip last
        thetas = np.arctan2(now[1] - last[1], now[0] - last[0])
        thetas = np.append(thetas, thetas[-1])  # last element gets last angle
        assert len(thetas) == len(posn_data.T)
        # append thetas to posn data
        posn_data = np.vstack([posn_data, thetas.T]).T
        # add the first position to the start of the data for the initial delay
        posn_data = np.insert(posn_data, [0], posn_data[0], axis=0)
        return posn_data

    @staticmethod
    def gather_posn_data_vec(ped_i: int, offset: Tuple[int, int, int]) -> List[float]:
        # old vectorized function for experimentation
        xy_data = np.vstack([ped_i.x, ped_i.y]).T
        s = np.sin(offset[2])
        c = np.cos(offset[2])

        # apply the rotations to the x, y positions
        x_rot = xy_data[:, 0] * c - xy_data[:, 1] * s + offset[0]
        y_rot = xy_data[:, 0] * s + xy_data[:, 1] * c + offset[1]
        xy_rot = np.vstack([x_rot, y_rot]).T

        # append vector angles for all the agents
        xy_rot_diff = np.diff(xy_rot, axis=0)
        thetas = np.arctan2(xy_rot_diff[:, 1], xy_rot_diff[:, 0])
        thetas = np.hstack((thetas, thetas[-1]))
        xytheta = np.vstack((xy_rot.T, thetas)).T

        return [xytheta[0]] + xytheta

    @staticmethod
    def gather_vel_data(
        time_data: List[float], posn_data: List[List[float]]
    ) -> List[List[float]]:
        # return linear speed to the list of variables
        pos_v_data: List[List[float]] = []
        assert len(time_data) == len(posn_data)
        for j, pos_2 in enumerate(posn_data):
            if j > 1:
                last_pos_2 = posn_data[j - 1]
                # calculating euclidean dist / delta_t
                delta_t = time_data[j] - time_data[j - 1]
                speed = euclidean_dist2(pos_2, last_pos_2) / delta_t
                v_x = speed * np.cos(pos_2[2])
                v_y = speed * np.sin(pos_2[2])
                pos_v_data.append(list(pos_2[:3]) + [v_x, v_y])
            else:
                pos_v_data.append(list(pos_2[:3]) + [0,0])  # initial speed is 0
        return pos_v_data

if __name__ == "__main__":
    test_w = PedTrajectoryDatasetWrapper(None,{'ped_traj_dataset':"/home/ustc/qiuqc/drlnav_master/drlnav_frame/USTC_lab/env/drlnav_env/envs/cfg/ped_dataset_cfg/datasets/eth/world_coordinate_inter_eth.csv"})
    test_w._generate_humans(0, 10)