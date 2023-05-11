import numpy as np
from typing import Dict, List


class TrajectoryPathHelper():
    """
    statistics for path
    """

    v_array = []
    w_array = []

    "path terms"
    w_zero = 0 # the times of w changes its sign. (+ -> - || - > +)
    v_jerk = 0 # average jerk (time derivative of acceleration) of the robot over its entire trajectory
    w_jerk = 0
    w_variance = 0 # variance of w
    w_avg = 0
    v_avg = 0
    w_acc = 0
    v_acc = 0

    def __init__(self, dt):
        self.dt = dt

    def add_vw(self, v, w):
        self.v_array.append(v)
        self.w_array.append(w)

    def get_steps(self):
        return len(self.v_array)

    def get_path_time(self):
        return round(len(self.v_array) * self.dt, 4)

    def get_w_zero(self):
        return self.w_zero

    def get_w_avg(self):
        return round(self.w_avg, 4)

    def get_v_avg(self):
        return round(self.v_avg, 4)

    def get_v_acc(self):
        return round(self.v_acc, 4)

    def get_w_acc(self):
        return round(self.w_acc, 4)

    def get_w_variance(self):
        return round(self.w_variance, 4)

    def get_w_jerk(self):
        return round(self.w_jerk, 4)

    def get_v_jerk(self):
        return round(self.v_jerk, 4)

    def cal_w_variance(self):
        self.w_variance = np.var(self.w_array)
        return round(self.w_variance, 4)

    def cal_w_avg(self):
        self.w_avg = np.average(np.abs(self.w_array))
        return round(self.w_avg, 4)

    def cal_v_avg(self):
        self.v_avg = np.average(self.v_array)
        return round(self.v_avg, 4)

    def cal_w_zero(self):
        tmp = 0
        w_zero = 0
        for w in self.w_array:
            if w == 0:
                if tmp != 0:
                    w_zero += 1
            else:
                if (w>0 and tmp < 0) or (w<0 and tmp>0):
                    w_zero += 1
            tmp = w
        self.w_zero = w_zero
        return w_zero

    def cal_jerk(self):
        dt = self.dt
        # print(self.w_array)
        # v_jrk
        v_acc = np.diff(self.v_array, axis=0) / dt
        v_jrk = np.diff(v_acc, axis=0) / dt
        self.v_jerk = np.average(np.abs(v_jrk))  #  np.average(np.square(v_jrk))
        self.v_acc = np.average(np.abs(v_acc))
        # w_jerk
        w_acc = np.diff(self.w_array, axis=0) / dt
        w_jrk = np.diff(w_acc, axis=0) / dt
        self.w_jerk = np.average(np.abs(w_jrk))  #  np.average(np.square(w_jrk))
        self.w_acc = np.average(np.abs(w_acc))
        # print('w_jerk',self.w_jerk)
        return self.v_jerk, self.w_jerk

    def get_metric_dict(self) -> Dict:
        return {
            "v_avg": self.get_v_avg(),
            "w_avg": self.get_w_avg(),
            "v_acc": self.get_v_acc(),
            "w_acc": self.get_w_acc(),
            "v_jerk": self.get_v_jerk(),
            "w_jerk": self.get_w_jerk(),
            "w_zero": self.get_w_zero(),
            "path_time": self.get_path_time(),
            "steps": self.get_steps(),
        }

    def clear_vw_array(self):
        self.v_array.clear()
        self.w_array.clear()
        self.w_zero = 0
        self.w_variance = 0
        self.w_avg = 0
        self.v_avg = 0
        self.w_jerk = 0
        self.v_jerk = 0
        self.v_acc = 0
        self.w_acc = 0

    def reset(self):
        self.cal_w_variance()
        self.cal_w_zero()
        self.cal_jerk()
        self.cal_v_avg()
        self.cal_w_avg()
        return