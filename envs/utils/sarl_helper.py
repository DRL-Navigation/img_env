import math
import numpy as np



def concate_sarl_states(original_state, ped_info, robot_radius, ped_vec_dim):
    # input information is one robot
    pgx, pgy, _, v, w = original_state
    pgd = math.sqrt(pgx ** 2 + pgy ** 2)
    v_pref = 0.6
    r_robot = robot_radius

    ped_num = int(ped_info[0])
    if ped_num == 0:
        # sarl ped num = 0 is not legal.
        ped_state = np.zeros([1, 7 + ped_vec_dim], dtype=np.float32)
        ped_state[0, :7] = pgx, pgy, v, w, pgd, v_pref, r_robot
        ped_state[0, 7:] = 10, 10, 0.5, 0.5, 0.2, 0.2+robot_radius, 14
        return ped_state
    else:
        ped_state = np.zeros([ped_num, 7 + ped_vec_dim], dtype=np.float32)
        """
        ped_tmp[j*7+1]=rt.px
        ped_tmp[j*7+2]=rt.py
        ped_tmp[j*7+3]=rt.vx
        ped_tmp[j*7+4]=rt.vy 
        ped_tmp[j*7+5]=rt.r_
        ped_tmp[j*7+6]=rt.r_+0.17
        ped_tmp[j*7+7]=math.sqrt(rt.px**2+rt.py**2)
        """
        for j in range(ped_num):
            ped_state[j, :7] = pgx, pgy, v, w, pgd, v_pref, r_robot
            ped_state[j, 7:] = ped_info[j * ped_vec_dim + 1: j * ped_vec_dim + 1 + ped_vec_dim]
            # #print("ped_state", ped_state)
            # sarl_states.append(ped_state)
        return ped_state
