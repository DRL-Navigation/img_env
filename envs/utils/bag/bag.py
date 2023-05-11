# coding=utf-8
import sys

sys.path.append(sys.path[0]+"/../../../")
import subprocess
import rospy
import os
import signal
import psutil
import rosbag
import tf
import math
import os.path as osp, time, atexit, os
import time
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.colors as mcolors
import atexit
import copy
import cv2


from cv_bridge import CvBridge, CvBridgeError
from tf2_msgs.msg import TFMessage
from matplotlib.patches import Circle, Wedge, Polygon, Ellipse
from matplotlib.collections import PatchCollection

from envs.utils import ros_utils as util


color2num = dict(
    gray=30,
    red=31,
    green=32,
    yellow=33,
    blue=34,
    magenta=35,
    cyan=36,
    white=37,
    crimson=38
)


def colorize(string, color, bold=False, highlight=False):
    attr = []
    num = color2num[color]
    if highlight: num += 10
    attr.append(str(num))
    if bold: attr.append('1')
    return '\x1b[%sm%s\x1b[0m' % (';'.join(attr), string)


def terminate_process_and_children(p):
    process = psutil.Process(p.pid)
    for sub_process in process.children(recursive=True):
        sub_process.send_signal(signal.SIGINT)
    p.wait()


def star_points(origin, r1, r2):
    ps = []
    for i in range(5):
        ps.append([r1 * math.cos(2 * math.pi * i / 5 + math.pi / 2) + origin[0],
                   r1 * math.sin(2 * math.pi * i / 5 + math.pi / 2) + origin[1]])
        ps.append([r2 * math.cos(2 * math.pi * i / 5 + math.pi / 2 + math.pi / 5) + origin[0],
                   r2 * math.sin(2 * math.pi * i / 5 + math.pi / 2 + math.pi / 5) + origin[1]])
    return ps


def dist2(p1, p2):
    return math.sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) +
                     (p1[1] - p2[1]) * (p1[1] - p2[1]))


def darker(color, percent):
    '''assumes color is rgb between (0, 0, 0) and (255, 255, 255)'''
    color = np.array(color)
    white = np.array([1, 1, 1])
    vector = white - color
    return color + vector * (1 - percent)


class BagRecorder(object):
    def __init__(self, output_path=""):
        self.output_path = output_path
        # if osp.exists(self.output_dir):
        #     print("Warning: Log dir %s already exists!" % self.output_dir)
        # else:
        #     os.makedirs(self.output_dir)
        self.cur_bag_process = None

    def robot_name(self, i, base_name='turtlerobot'):
        return base_name + 'x' * (i // 10) + str(i % 10)

    def record(self, topics='small', robot_name='turtlerobot', robot_total=4):
        if topics == 'all':
            topics = '-a'
        elif topics == 'small':
            all_cmd_vel = ''
            all_odom = ''
            all_goal = ''
            for i in range(robot_total):
                all_odom += ' ' + self.robot_name(i) + '/odom'
                all_cmd_vel += ' ' + self.robot_name(i) + '/cmd_vel'
                all_goal += ' ' + self.robot_name(i) + '/goal'
            topics = '/tf' + all_cmd_vel + all_odom + all_goal + ' /test_states' + ' /world_obs'
        cmd_str = "rosbag record " + topics + " -O " + self.output_path
        print("cmd", cmd_str)
        print("start record bag {} in {}".format(topics, self.output_path), flush=True)
        self.cur_bag_process = subprocess.Popen(cmd_str, shell=True)
        print("record pid: ", self.cur_bag_process.pid, flush=True)
        time.sleep(5.0)
        return self.cur_bag_process

    def stop(self, p=None):
        if p is None:
            p = self.cur_bag_process
        terminate_process_and_children(p)


class BagReader(object):
    def __init__(self, args):
        super(BagReader, self).__init__()
        self.tf_tree = tf.Transformer(True, rospy.Duration(3600.0))
        self.bag_name = args.file
        print("read bag file ...")
        self.bag = rosbag.Bag(self.bag_name)
        atexit.register(self.bag.close)
        self.robot_base_name = "turtlerobot"
        self.plot_colors = list(mcolors.TABLEAU_COLORS)
        self.plot_colors[3], self.plot_colors[-1] = self.plot_colors[-1], self.plot_colors[3]
        self.max_v = 0.6
        self.reach_th = 0.3
        self.bridge = CvBridge()

        self.robot_radius = float(args.robot_radius)
        self.topic = args.topic

    def read_tf_tree(self):
        self.type = 'gazebo'
        print("read_tf_tree ...")
        for topic, msg, t in self.bag.read_messages(topics=['/tf']):
            for msg_tf in msg.transforms:
                self.tf_tree.setTransform(msg_tf)

    def get_transform(self, from_frame, to_frame, timestamp):
        try:
            (translation,
             quaternion) = self.tf_tree.lookupTransform(
                to_frame, from_frame, timestamp)
            return util.matrix_from_t_q(translation, quaternion)
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            rospy.logerr("listen tf from %s to %s error", from_frame, to_frame)

    def robot_name(self, i, base_name='turtlerobot'):
        return base_name + 'x' * (i // 10) + str(i % 10)

    def robot_index(self, robot_name, base_name='turtlerobot'):
        robot_index = robot_name.split(base_name)[1]
        x_total = robot_index.count('x')
        if x_total != 0:
            robot_index = x_total * 10 + int(robot_index.split('x' * x_total)[1])
        return int(robot_index)

    def image_trans(self, img_ros):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_ros, desired_encoding="8UC1")
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)
        except CvBridgeError as e:
            print(e)
        return cv_image

    def image_trans_rgb(self, img_ros):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_ros, desired_encoding="8UC3")
        #   cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)
        except CvBridgeError as e:
            print(e)
        return cv_image

    def pose_msg_list(self, pose_msg, tf_map_plot):
        tf_robot_map = util.matrix_from_t_q([pose_msg.position.x, pose_msg.position.y, 0],
                                            [pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z,
                                             pose_msg.orientation.w])
        tf_robot_plot = util.mul_matrix(tf_map_plot, tf_robot_map)
        t = util.t_from_matrix(tf_robot_plot)
        rpy = util.rpy_from_matrix(tf_robot_plot)
        return [t[0], t[1], rpy[2]]

    def distill_img_env_bag(self, interval=1):
        self.type = 'image'
        summary = []
        cur_time = 0
        for topic, msg, t in self.bag.read_messages(topics=[self.topic]):
            ep = {}
            ep['begin_time'] = cur_time
            ep['obs'] = {}
            ep['robot'] = {}
            ep['ped'] = {}
            ep['obs_map'] = self.image_trans(msg.obs_map)
            #ep['ped_map'] = self.image_trans_rgb(msg.ped_map)
            ep['resolution'] = msg.resolution
            ep['env_name'] = msg.env_name
            ep['step_hz'] = msg.step_hz
            tf_map_plot = util.matrix_from_t_y([0, ep['obs_map'].shape[0] * ep['resolution'], 0], -1.570796)
            # 为了和gazebo环境bag坐标系对应，但会影响静态地图绘制, TODO
            # tf_map_plot = util.mul_matrix(util.matrix_from_t_y([-0.5,-0.5,0], 1.5708), tf_map_plot)
            robots_res = msg.robots_res
            max_end_time = -1
            for i in range(len(robots_res)):
                ep['robot'][i] = {}
                ep['robot'][i]['result'] = robots_res[i].result
                ep['robot'][i]['control_vs'] = robots_res[i].vs
                ep['robot'][i]['control_ws'] = robots_res[i].ws
                ep['robot'][i]['control_beep'] = robots_res[i].v_ys
                ep['robot'][i]['odom_vs'] = robots_res[i].vs
                ep['robot'][i]['odom_ws'] = robots_res[i].ws
                ep['robot'][i]['init_pose'] = self.pose_msg_list(robots_res[i].info.init_pose, tf_map_plot)
                ep['robot'][i]['goal'] = util.transform_point(tf_map_plot,
                                                              [robots_res[i].info.goal.x, robots_res[i].info.goal.y,
                                                               0])[0:2]
                ep['robot'][i]['shape'] = robots_res[i].info.shape
                ep['robot'][i]['size'] = robots_res[i].info.size
                ep['robot'][i]['poses'] = []
                ep['robot'][i]['pose_times'] = []
                ep['robot'][i]['bbx'] = []
                for j in range(len(robots_res[i].poses)):
                    ep['robot'][i]['poses'].append(self.pose_msg_list(robots_res[i].poses[j], tf_map_plot))
                    ep['robot'][i]['pose_times'].append(cur_time + j * ep['step_hz'])
                    if ep['robot'][i]['shape'] == 'rectangle':
                        tf_robot_plot = util.matrix_from_t_y(
                            [ep['robot'][i]['poses'][-1][0], ep['robot'][i]['poses'][-1][1], 0],
                            ep['robot'][i]['poses'][-1][2])
                        tmp = []
                        tmp.append(util.transform_point(tf_robot_plot,
                                                        [ep['robot'][i]['size'][0], ep['robot'][i]['size'][3], 0])[0:2])
                        tmp.append(util.transform_point(tf_robot_plot,
                                                        [ep['robot'][i]['size'][1], ep['robot'][i]['size'][3], 0])[0:2])
                        tmp.append(util.transform_point(tf_robot_plot,
                                                        [ep['robot'][i]['size'][1], ep['robot'][i]['size'][2], 0])[0:2])
                        tmp.append(util.transform_point(tf_robot_plot,
                                                        [ep['robot'][i]['size'][0], ep['robot'][i]['size'][2], 0])[0:2])
                        ep['robot'][i]['bbx'].append(tmp)
                    if ep['robot'][i]['shape'] == 'circle':
                        tmp = []
                        tmp.append(ep['robot'][i]['poses'][-1])
                        tmp.append(ep['robot'][i]['size'][2])
                        ep['robot'][i]['bbx'].append(tmp)
                ep['robot'][i]['odom_times'] = ep['robot'][i]['pose_times'][:-1]
                ep['robot'][i]['control_times'] = ep['robot'][i]['pose_times'][:-1]
                distance = 0
                for j in range(len(ep['robot'][i]['poses']) - 1):
                    p1 = ep['robot'][i]['poses'][j]
                    p2 = ep['robot'][i]['poses'][j + 1]
                    dist = math.sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]))
                    distance += dist
                ep['robot'][i]['distance'] = distance
                ep['robot'][i]['end_time'] = cur_time + len(ep['robot'][i]['control_times']) * ep['step_hz']
                if ep['robot'][i]['end_time'] > max_end_time:
                    max_end_time = ep['robot'][i]['end_time']

            cur_time = 0
            peds_res = msg.peds_res
            for i in range(len(peds_res)):
                ep['ped'][i] = {}
                ep['ped'][i]['init_pose'] = self.pose_msg_list(peds_res[i].info.init_pose, tf_map_plot)
                ep['ped'][i]['goal'] = util.transform_point(tf_map_plot,
                                                            [peds_res[i].info.goal.x, peds_res[i].info.goal.y, 0])[0:2]
                ep['ped'][i]['shape'] = peds_res[i].info.shape
                ep['ped'][i]['size'] = peds_res[i].info.size
                ep['ped'][i]['poses'] = []
                ep['ped'][i]['pose_times'] = []
                ep['ped'][i]['bbx'] = []
                for j in range(len(peds_res[i].poses) - 1):
                    ep['ped'][i]['poses'].append(self.pose_msg_list(peds_res[i].poses[j + 1], tf_map_plot))
                    ep['ped'][i]['poses'][-1][-1] = math.atan2(peds_res[i].v_ys[j], peds_res[i].vs[j])
                    ep['ped'][i]['pose_times'].append(cur_time + j * ep['step_hz'])
                    if ep['ped'][i]['shape'] == 'circle':
                        tmp = []
                        tmp.append(ep['ped'][i]['poses'][-1])
                        tmp.append(ep['ped'][i]['size'][2])
                        ep['ped'][i]['bbx'].append(tmp)

            summary.append(copy.deepcopy(ep))
        return summary

    def get_episode_time(self):
        print("get_episode_time ...")
        test_summary = {}
        cur_ep = 0
        for topic, msg, t in self.bag.read_messages(topics=['/test_states']):
            if 'ep' in msg.data and 'begin' in msg.data:
                cur_ep = int(msg.data.split('ep')[1].split(' begin')[0])
                test_summary[cur_ep] = {}
                test_summary[cur_ep]['robot'] = {}
                test_summary[cur_ep]['begin_time'] = t.to_sec()
                test_summary[cur_ep]['obs'] = {}
            if 'robot' in msg.data and 'end' in msg.data:
                robot, res = msg.data.split('robot')[1].split(' end, ')
                x_total = robot.count('x')
                if x_total != 0:
                    robot = x_total * 10 + int(robot.split('x' * x_total)[1])
                test_summary[cur_ep]['robot'][int(robot)] = {}
                test_summary[cur_ep]['robot'][int(robot)]['end_time'] = t.to_sec()
                test_summary[cur_ep]['robot'][int(robot)]['result'] = res
                test_summary[cur_ep]['robot'][int(robot)]['poses'] = []
                test_summary[cur_ep]['robot'][int(robot)]['pose_times'] = []
                test_summary[cur_ep]['robot'][int(robot)]['control_times'] = []
                test_summary[cur_ep]['robot'][int(robot)]['control_vs'] = []
                test_summary[cur_ep]['robot'][int(robot)]['control_ws'] = []
                test_summary[cur_ep]['robot'][int(robot)]['control_beep'] = []
                test_summary[cur_ep]['robot'][int(robot)]['odom_times'] = []
                test_summary[cur_ep]['robot'][int(robot)]['odom_vs'] = []
                test_summary[cur_ep]['robot'][int(robot)]['odom_ws'] = []
        print("============ Bag Summary ===========")
        print('Number of episodes: {0}'.format(len(test_summary)))
        print('Number of robots: {0}'.format(len(test_summary[0]['robot'])))
        # for i in range(len(test_summary)):
        #     print 'episode {0}:'.format(i)
        #     print '  begin_time: {0}'.format(test_summary[i]['begin_time'])
        #     for j in range(len(test_summary[i]['robot'])):
        #         print '  robot {0}:'.format(j)
        #         print '    end_time: {0}'.format(test_summary[i]['robot'][j]['end_time'])
        #         print '    result: ' + test_summary[i]['robot'][j]['result']
        # print 'Number of episodes: {0}'.format(len(test_summary))
        # print 'Number of robots: {0}'.format(len(test_summary[0]['robot']))
        print("====================================")
        return test_summary

    def distill_world(self, summary=None):
        print("distill_world ...")
        if summary is None:
            summary = self.get_episode_time()
        ep_total = len(summary)
        i = 0
        for topic, msg, t in self.bag.read_messages(topics=['/world_obs']):
            summary[i]['obs']['shapes'] = []
            summary[i]['obs']['bbs'] = []
            for j in range(len(msg.name)):
                name = msg.name[j]
                shape = gz_util.get_model_shape(name)
                if shape != None:
                    pose = msg.pose[j]
                    for s in range(len(shape['component_shape'])):
                        summary[i]['obs']['shapes'].append(shape['component_shape'][s])
                        if shape['component_shape'][s] == 'circle':
                            summary[i]['obs']['bbs'].append(
                                [pose.position.x, pose.position.y, shape['bounding_box'][s]])
                        elif shape['component_shape'][s] == 'polygon':
                            tf_self_world = ros_util.matrix_from_t_q([pose.position.x, pose.position.y, 0],
                                                                     [pose.orientation.x, pose.orientation.y,
                                                                      pose.orientation.z, pose.orientation.w])
                            point_self = shape['bounding_box'][s]
                            tmp = []
                            for p in point_self:
                                tmp.append(ros_util.transform_point(tf_self_world, p[:2] + [0])[:2])
                            summary[i]['obs']['bbs'].append(tmp[:])
            i += 1
        return summary

    def distill_trajectory(self, interval=0.2, summary=None):
        print("distill_trajectory ...")
        if summary is None:
            summary = self.distill_control_vw()
        ep_total = len(summary)
        robot_total = len(summary[0]['robot'])
        cur_ep = 0
        cur_time = summary[0]['robot'][0]['control_times'][0]
        is_end = [0] * robot_total
        while True:
            if is_end.count(1) == robot_total:
                cur_ep += 1
                if cur_ep >= ep_total:
                    break
                cur_time = summary[cur_ep]['robot'][0]['control_times'][0]
                is_end = [0] * robot_total

            for i in range(robot_total):
                end_time = summary[cur_ep]['robot'][i]['end_time']
                if cur_time <= end_time:
                    robot_frame = '/' + self.robot_name(i, self.robot_base_name) + '/base_link'
                    tf_m = self.get_transform(robot_frame, '/odom', rospy.Time(cur_time))
                    if tf_m is not None:
                        robot_t = util.t_from_matrix(tf_m)
                        robot_rpy = util.rpy_from_matrix(tf_m)
                        robot_pose = [robot_t[0], robot_t[1], robot_rpy[2]]
                        summary[cur_ep]['robot'][i]['poses'].append(robot_pose[:])
                        summary[cur_ep]['robot'][i]['pose_times'].append(cur_time)
                elif abs(cur_time - end_time) <= interval:
                    robot_frame = '/' + self.robot_name(i, self.robot_base_name) + '/base_link'
                    tf_m = self.get_transform(robot_frame, '/odom', rospy.Time(cur_time))
                    if tf_m is not None:
                        robot_t = util.t_from_matrix(tf_m)
                        robot_rpy = util.rpy_from_matrix(tf_m)
                        robot_pose = [robot_t[0], robot_t[1], robot_rpy[2]]
                        if dist2(robot_pose, summary[cur_ep]['robot'][i]['poses'][-1]) <= interval * self.max_v:
                            summary[cur_ep]['robot'][i]['poses'].append(robot_pose[:])
                            summary[cur_ep]['robot'][i]['pose_times'].append(cur_time)
                else:
                    is_end[i] = 1
            cur_time += interval
        return summary

    def cal_vw(self, summary):
        print(summary)
        ep_total = len(summary)
        robot_total = len(summary[0]['robot'])
        for ep in range(ep_total):
            for i in range(robot_total):
                poses = summary[ep]['robot'][i]['poses']
                pose_times = summary[ep]['robot'][i]['pose_times']
                pose_vs = []
                pose_ws = []
                dxs = []
                dys = []
                das = []
                dts = []
                dists = []
                for j in range(len(poses) - 1):
                    tf_1 = util.matrix_from_t_y([poses[j][0], poses[j][1], 0], poses[j][2])
                    tf_2 = util.matrix_from_t_y([poses[j + 1][0], poses[j + 1][1], 0], poses[j + 1][2])
                    tf_change = util.mul_matrix(util.inverse(tf_1), tf_2)
                    translation = util.t_from_matrix(tf_change)
                    rpy = util.rpy_from_matrix(tf_change)
                    dxs.append(translation[0])
                    dys.append(translation[1])
                    das.append(rpy[2])
                    dts.append(pose_times[j + 1] - pose_times[j])
                all_dist = 0
                for k in range(len(dxs)):
                    dist = math.sqrt(dxs[k] * dxs[k] + dys[k] * dys[k])
                    # print "dist", dist
                    # print "exa", dist2(poses[k], poses[k+1])
                    all_dist += dist
                    dists.append(dist)
                    pose_vs.append(dist / dts[k])
                    pose_ws.append(das[k] / dts[k])
                summary[ep]['robot'][i]['pose_vs'] = pose_vs[:]
                summary[ep]['robot'][i]['pose_ws'] = pose_ws[:]
                summary[ep]['robot'][i]['distance'] = np.array(dists).sum()
                # print "sum ", np.array(dists).sum()
                # print "real ", np.array(dists).sum() - dist2(poses[0], poses[-1])
                # print all_dist
        return summary

    def _plot_t_ep(self, summary, episode, ax, xy_lim, obs_color, robot_every_circle=True):
        star_radius1 = self.robot_radius / 1.8
        star_radius2 = star_radius1 / 3
        ep_total = len(summary)
        robot_total = len(summary[0]['robot'])
        ped_total = len(summary[0]['ped'])
        plot_colors = self.plot_colors * int(math.ceil((robot_total + 2 * ped_total) / float(len(self.plot_colors))))
        keep_away_goal_dist = self.reach_th / 2
        if summary[episode]['obs'].get('shapes'):
            for s in range(len(summary[episode]['obs']['shapes'])):
                shape = summary[episode]['obs']['shapes'][s]
                bb = summary[episode]['obs']['bbs'][s]
                if shape == "circle":
                    ax.add_artist(Circle((bb[0], bb[1]), bb[2], color=obs_color))
                elif shape == "polygon":
                    ax.add_artist(Polygon(bb, True, color=obs_color))
        max_step = 0
        for r in range(robot_total):
            if len(np.array(summary[episode]['robot'][r]['poses'])) > max_step:
                max_step = len(np.array(summary[episode]['robot'][r]['poses']))
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_xlim([2, 9])
        ax.set_ylim([2, 9])
        # ax.set_xlim([xy_lim[0], xy_lim[1]])
        # ax.set_ylim([xy_lim[2], xy_lim[3]])
        ax.set_aspect('equal', 'box')
        if self.type == 'image':
            ax.imshow(summary[episode]['obs_map'],
                      extent=[0, summary[episode]['obs_map'].shape[1] * summary[episode]['resolution'], \
                              0, summary[episode]['obs_map'].shape[0] * summary[episode]['resolution']])
        elif self.type == 'image_ped':
            ax.imshow(summary[episode]['obs_map'],
                      extent=[0, summary[episode]['obs_map'].shape[1] * summary[episode]['resolution'], \
                              0, summary[episode]['obs_map'].shape[0] * summary[episode]['resolution']])
        for i in range(max_step):
            for r in range(robot_total):
                poses = np.array(summary[episode]['robot'][r]['poses'])
                beeps = np.array(summary[episode]['robot'][r]['control_beep'])
                pose_len = len(poses)
                plot_xs = poses[:, 0]
                plot_ys = poses[:, 1]
                plot_xs = plot_xs.tolist()
                plot_ys = plot_ys.tolist()

                alpha = np.linspace(0.2, 0.8, len(plot_xs) + 1)
                alpha = alpha[1:len(plot_xs) + 1]
                if i < len(plot_xs):
                    if self.type == 'image' or self.type == 'image_ped':
                        if summary[episode]['robot'][r]['shape'] == 'rectangle':
                            ax.add_artist(plt.Polygon(summary[episode]['robot'][r]['bbx'][i], True, alpha=0.8,
                                                      color=darker(mcolors.to_rgb(plot_colors[r]), alpha[i]), ec=None))
                        if summary[episode]['robot'][r]['shape'] == 'circle':
                            # red collison
                            if i == len(plot_xs) - 1 and summary[episode]['robot'][r]['result'] == 'collision':
                                ax.add_artist(plt.Circle((plot_xs[i], plot_ys[i]), radius=self.robot_radius, alpha=0.8,
                                                         color=darker(mcolors.to_rgb(plot_colors[9]), alpha[i]),
                                                         ec=None))
                            else:
                                # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                                if robot_every_circle:
                                    ax.add_artist(plt.Circle((plot_xs[i], plot_ys[i]), radius=self.robot_radius, alpha=0.8,
                                                                 color=darker(mcolors.to_rgb(plot_colors[r]), alpha[i]),
                                                                 ec=None))
                                else:
                                    if i % 4 == 0:
                                        ax.add_artist(plt.Circle((plot_xs[i], plot_ys[i]), radius=self.robot_radius, alpha=0.8,
                                                                 color=darker(mcolors.to_rgb(plot_colors[r]), alpha[i]), ec=None))
                                    #
                                    # ax.add_artist(plt.Circle((plot_xs[i], plot_ys[i]), radius=0.03, alpha=1.0,
                                    #                              color=darker(mcolors.to_rgb(plot_colors[r]), alpha[i]),
                                    #                              ec=None))
                        # 画一个圆形的beep,
                        if i != len(beeps)and len(beeps) and beeps[i] == 1 :
                            ax.add_artist(plt.Circle((plot_xs[i], plot_ys[i]), radius=1.0, alpha=0.8,
                                                     color=darker(mcolors.to_rgb(plot_colors[r]), alpha[i]),
                                                     fill=False))
                    else:
                        ax.add_artist(plt.Circle((plot_xs[i], plot_ys[i]), radius=self.robot_radius, alpha=0.8,
                                                 color=darker(mcolors.to_rgb(plot_colors[r]), alpha[i]), ec=None))
            for r in range(ped_total):
                poses = np.array(summary[episode]['ped'][r]['poses'])
                pose_len = len(poses)
                plot_xs = poses[:, 0]
                plot_ys = poses[:, 1]
                plot_thetas = poses[:, 2]
                plot_xs = plot_xs.tolist()
                plot_ys = plot_ys.tolist()

                alpha = np.linspace(0.2, 0.8, len(plot_xs) + 1)
                alpha = alpha[1:len(plot_xs) + 1]
                if i < len(plot_xs):
                    if self.type == 'image' or self.type == 'image_ped':
                         # ax.add_artist(plt.Circle((plot_xs[i], plot_ys[i]), radius=robot_radius, alpha=1,
                        # color = darker(mcolors.to_rgb(plot_colors[r]), alpha[i]), ec=None))
                        # ax.add_artist(
                        #     Ellipse(xy=(plot_xs[i], plot_ys[i]), width=0.2, height=0.3, angle=plot_thetas[i] * 57.29,
                        #             facecolor=darker(mcolors.to_rgb(plot_colors[r + robot_total]), alpha[i]),
                        #             edgecolor=darker([0, 0.5, 1], alpha[i]), linewidth=0.2, alpha=0.8))
                        if i % 4 == 0 :
                            ax.add_artist(Circle(xy=(plot_xs[i], plot_ys[i]), radius=0.1, alpha=0.6,
                                                 facecolor=darker(mcolors.to_rgb(plot_colors[r + robot_total]),
                                                                  alpha[i]), edgecolor=darker([0, 0.5, 1], alpha[i]),
                                                 linewidth=0.2))

                        ax.add_artist(Circle(xy=(plot_xs[i], plot_ys[i]), radius=0.03, alpha=1.0,
                                                 facecolor=darker(
                                                     mcolors.to_rgb(plot_colors[r + robot_total]),
                                                     alpha[i]), edgecolor=darker([0, 0.5, 1], alpha[i]),
                                                 linewidth=0.2))

        for r in range(ped_total):
            poses = np.array(summary[episode]['ped'][r]['poses'])
            pose_len = len(poses)
            plot_xs = poses[:,0]
            plot_ys = poses[:,1]
            plot_xs = plot_xs.tolist()
            plot_ys = plot_ys.tolist()
            # goal = [(3*goal[0]+plot_xs[-1]) /4, (3 * goal[1]+plot_ys[-1]) /4]

            alpha = np.linspace(0.2, 0.8, len(plot_xs)+1)
            alpha = alpha[1:len(plot_xs)+1]
            # for i in range(len(plot_xs)):
            #     ax.add_artist(plt.Circle((plot_xs[i], plot_ys[i]), radius=robot_radius, alpha=1,
            #     color = darker(mcolors.to_rgb(plot_colors[r]), alpha[i]), ec=None))

            ax.add_artist(Polygon(star_points([plot_xs[-1], plot_ys[-1]], star_radius1, star_radius2), True,
                color=plot_colors[r + robot_total], linewidth=0.5, fill=True, hatch='/'))
            ax.plot(plot_xs, plot_ys, color = plot_colors[r + robot_total], linewidth=0.5, alpha=0.8)
        for r in range(robot_total):
            poses = np.array(summary[episode]['robot'][r]['poses'])
            goal = summary[episode]['robot'][r]['goal']
            pose_len = len(poses)
            plot_xs = poses[:, 0]
            plot_ys = poses[:, 1]
            plot_xs = plot_xs.tolist()
            plot_ys = plot_ys.tolist()
            # goal = [(3*goal[0]+plot_xs[-1]) /4, (3 * goal[1]+plot_ys[-1]) /4]

            alpha = np.linspace(0.2, 0.8, len(plot_xs) + 1)
            alpha = alpha[1:len(plot_xs) + 1]
            # for i in range(len(plot_xs)):
            #     ax.add_artist(plt.Circle((plot_xs[i], plot_ys[i]), radius=robot_radius, alpha=1,
            #     color = darker(mcolors.to_rgb(plot_colors[r]), alpha[i]), ec=None))

            if summary[episode]['robot'][r]['result'] == 'collision':
                ax.add_artist(Polygon(star_points(goal, star_radius1, star_radius2), True,
                                      color=plot_colors[r], linewidth=0.5, fill=True, hatch='/'))
                # ax.scatter([goal[0]], [goal[1]], s=star_size, marker='*', color = plot_colors[r], zorder=3)
                ax.plot(plot_xs, plot_ys, color=plot_colors[r], linewidth=0.5, alpha=0.8)
                ax.plot([poses[-1][0], goal[0]], [poses[-1][1], goal[1]], color='black',
                        linewidth=0.5, linestyle='dashed')
            elif summary[episode]['robot'][r]['result'] == 'stuck':
                ax.add_artist(Polygon(star_points(goal, star_radius1, star_radius2), True,
                                      color=plot_colors[r], linewidth=0.5, fill=True, hatch='/'))
                # ax.scatter([goal[0]], [goal[1]], s=star_size, marker='*', color = plot_colors[r], zorder=3)
                ax.plot(plot_xs, plot_ys, color=plot_colors[r], linewidth=0.5, alpha=0.8)
                ax.plot([poses[-1][0], goal[0]], [poses[-1][1], goal[1]], color='black',
                        linewidth=0.5)
            elif summary[episode]['robot'][r]['result'] == 'arrive':
                goal = [plot_xs[-1], plot_ys[-1]]
                ax.add_artist(Polygon(star_points(goal, star_radius1, star_radius2), True,
                                      color=plot_colors[r], linewidth=0.5, fill=True, hatch='/'))
                ax.plot(plot_xs, plot_ys, color=plot_colors[r], linewidth=0.5, alpha=0.8)
                # ax.scatter([poses[-1][0]], [poses[-1][1]], s=star_size, marker='*', color = plot_colors[r], zorder=3)

    def draw_trajectory(self, summary=None, ep_split=False, episode='all', save_dir=None, robot_every_circle=True):
        print ("draw_trajectory ...")
        if episode == 'split':
            ep_split = True
        robot_radius = self.robot_radius
        obs_color = 'black'
        if summary is None:
            summary = self.distill_control_vw()
            summary = self.distill_trajectory(summary)
            summary = self.distill_goal(summary)
        ep_total, robot_total = len(summary), len(summary[0]['robot'])
        all_poses = []
        for m in range(ep_total):
            for n in range(robot_total):
                all_poses += summary[m]['robot'][n]['poses']
        all_poses = np.array(all_poses)
        all_pose_xs = all_poses[:, 0]
        all_pose_ys = all_poses[:, 1]
        xy_lim = [int(math.floor(all_pose_xs.min() - robot_radius)), int(math.ceil(all_pose_xs.max() + robot_radius)), \
                  int(math.floor(all_pose_ys.min() - robot_radius)), int(math.ceil(all_pose_ys.max() + robot_radius))]
        # xy_lim = [-3, 3, -3, 3]
        if episode != 'all' and episode != 'split' and int(episode) < ep_total:
            episode = int(episode)
            plt.rc('font', family='serif', serif='Times')
            # plt.rc('text', usetex=True)
            plt.rc('xtick', labelsize=9)
            plt.rc('ytick', labelsize=9)
            plt.rc('axes', labelsize=10)
            plt.rc('legend', fontsize=10)
            fig, ax = plt.subplots()
            plt.xticks(range(2, 9 + 1))
            plt.yticks(range(2, 9 + 1))
            # ax.set_xticklabels(range(0, xy_lim[1] + 1 - xy_lim[0]))
            # ax.set_yticklabels(range(0, xy_lim[3] + 1 - xy_lim[2]))
            fig.subplots_adjust(left=.15, bottom=.15, right=.97, top=.97)
            # fig.suptitle('{0} Robot Trajectories in Episode {1}'.format(robot_total, episode), fontsize=14)
            self._plot_t_ep(summary, episode, ax, xy_lim, robot_radius, obs_color, robot_every_circle)
            width_x = 3.487
            width_y = 3.487
            fig.set_size_inches(width_x, width_y)
            # plt.savefig(osp.join(save_dir, 'ep' + str(episode) + '.png'), dpi=100)
            plt.savefig(osp.join(save_dir, 'ep' + str(episode) + '.pdf'), dpi=200)
            return
        if ep_split:
            for ep in range(ep_total):
                plt.rc('font', family='serif', serif='Times')
                plt.rc('text', usetex=True)
                plt.rc('xtick', labelsize=9)
                plt.rc('ytick', labelsize=9)
                plt.rc('axes', labelsize=10)
                plt.rc('legend', fontsize=10)
                fig, ax = plt.subplots()
                print (xy_lim)
                plt.xticks(range(xy_lim[0], xy_lim[1] + 1))
                plt.yticks(range(xy_lim[2], xy_lim[3] + 1))
                ax.set_xticklabels(range(0, xy_lim[1] + 1 - xy_lim[0]))
                ax.set_yticklabels(range(0, xy_lim[3] + 1 - xy_lim[2]))
                fig.subplots_adjust(left=.15, bottom=.15, right=.97, top=.97)
                # fig.suptitle('{0} Robot Trajectories in Episode {1}'.format(robot_total, ep), fontsize=14)
                self._plot_t_ep(summary, ep, ax, xy_lim, robot_radius, obs_color, robot_every_circle)
                width_x = 3.487
                width_y = 3.487
                fig.set_size_inches(width_x, width_y)
                plt.savefig(osp.join(save_dir, str(time.time()) + '_ep' + str(ep) + '.png'), dpi=100)
        else:
            max_column = 5
            nrows = int(math.ceil(ep_total / 5.0))
            ncols = min(max_column, ep_total)
            fig, ax = plt.subplots(nrows=nrows, ncols=ncols, constrained_layout=True)
            width_x = 0.35 * (xy_lim[1] - xy_lim[0] + 5)
            width_y = 0.35 * (xy_lim[3] - xy_lim[2] + 5)
            fig.set_size_inches(width_x * ncols, width_y * nrows)
            for i, axi in enumerate(ax.flat):
                print (i)
                if i >= ep_total:
                    break
                rowid = i // ncols
                colid = i % ncols
                self._plot_t_ep(summary, i, axi, xy_lim, obs_color, robot_every_circle)
                # axi.text(xy_lim[0] + 0.5, xy_lim[3] - 0.5, 'ep ' + str(i))
            if save_dir is None:
                save_dir = '../'
            plt.savefig(osp.join(save_dir, str(time.time()) + '_all.png'), dpi=100)

    def distill_goal(self, summary=None):
        print("distill_goal ...")
        if summary is None:
            summary = self.get_episode_time()
        ep_total = len(summary)
        robot_total = len(summary[0]['robot'])
        goal_topics = []
        for i in range(robot_total):
            goal_topics.append(self.robot_name(i, self.robot_base_name) + '/goal')
        cur_ep = 0
        is_end = [0] * robot_total
        for topic, msg, t in self.bag.read_messages(topics=goal_topics):
            robot_name = topic.split('/goal')[0]
            robot_index = self.robot_index(robot_name, self.robot_base_name)
            if is_end.count(1) == robot_total:
                cur_ep += 1
                if cur_ep >= ep_total:
                    break
                is_end = [0] * robot_total
            if msg.pose.position.z != -1.0:
                summary[cur_ep]['robot'][robot_index]['goal'] = [msg.pose.position.x, msg.pose.position.y]
                is_end[robot_index] = 1
        return summary

    def distill_control_vw(self, summary=None):
        print ("distill_control_vw ...")
        if summary is None:
            summary = self.get_episode_time()
        ep_total = len(summary)
        robot_total = len(summary[0]['robot'])
        control_topics = []
        for i in range(robot_total):
            control_topics.append(self.robot_name(i, self.robot_base_name) + '/cmd_vel')
        cur_ep = 0
        is_end = [0] * robot_total
        for topic, msg, t in self.bag.read_messages(topics=control_topics):
            robot_name = topic.split('/cmd_vel')[0]
            robot_index = self.robot_index(robot_name, self.robot_base_name)
            if is_end.count(1) == robot_total:
                cur_ep += 1
                if cur_ep >= ep_total:
                    break
                is_end = [0] * robot_total
            cur_ep_time = summary[cur_ep]['begin_time']
            cur_t = t.to_sec()
            if cur_t >= cur_ep_time:
                if cur_t <= summary[cur_ep]['robot'][robot_index]['end_time']:
                    summary[cur_ep]['robot'][robot_index]['control_times'].append(cur_t)
                    summary[cur_ep]['robot'][robot_index]['control_vs'].append(msg.linear.x)
                    summary[cur_ep]['robot'][robot_index]['control_ws'].append(msg.angular.z)
                else:
                    is_end[robot_index] = 1
        return summary

    def distill_vw_from_odom(self, summary=None):
        print ("distill_vw_from_odom ...")
        if summary is None:
            summary = self.get_episode_time()
        ep_total = len(summary)
        robot_total = len(summary[0]['robot'])
        odom_topics = []
        for i in range(robot_total):
            odom_topics.append(self.robot_name(i, self.robot_base_name) + '/odom')
        cur_ep = 0
        is_end = [0] * robot_total
        for topic, msg, t in self.bag.read_messages(topics=odom_topics):
            robot_name = topic.split('/odom')[0]
            robot_index = self.robot_index(robot_name, self.robot_base_name)
            if is_end.count(1) == robot_total:
                cur_ep += 1
                if cur_ep >= ep_total:
                    break
                is_end = [0] * robot_total
            cur_ep_time = summary[cur_ep]['robot'][robot_index]['control_times'][0]
            cur_t = t.to_sec()
            if cur_t >= cur_ep_time:
                if cur_t <= summary[cur_ep]['robot'][robot_index]['end_time']:
                    summary[cur_ep]['robot'][robot_index]['odom_times'].append(cur_t)
                    summary[cur_ep]['robot'][robot_index]['odom_vs'].append(msg.twist.twist.linear.x)
                    summary[cur_ep]['robot'][robot_index]['odom_ws'].append(msg.twist.twist.angular.z)
                else:
                    is_end[robot_index] = 1
        return summary

    def draw_vw(self, data_source, summary=None, episode='all', robot=None):
        print ("draw_vw ...")
        if data_source == 'odom' or data_source == 'vw':
            if summary is None:
                summary = self.distill_control_vw()
                summary = self.distill_vw_from_odom(summary=summary)
            x_key = 'odom_times'
            v_key = 'odom_vs'
            w_key = 'odom_ws'
        elif data_source == 'control':
            if summary is None:
                summary = self.distill_control_vw()
            x_key = 'control_times'
            v_key = 'control_vs'
            w_key = 'control_ws'
        ep_total = len(summary)
        robot_total = len(summary[0]['robot'])
        if episode != 'all' and int(episode) < ep_total:
            episode = int(episode)
            if data_source == 'vw' and robot != None and int(robot) < robot_total:
                robot = int(robot)
                fig, ax = plt.subplots(nrows=2, ncols=2, constrained_layout=True)
                fig.suptitle('Robot Velocities of (Episode {0}, Robot {1})'.format(episode, robot), fontsize=14)
                for i, axi in enumerate(ax.flat):
                    rowid = i // 2
                    colid = i % 2
                    if rowid == 0 and colid == 0:
                        axi.plot(summary[episode]['robot'][robot]['control_times'],
                                 summary[episode]['robot'][robot]['control_vs'], marker="o")
                        axi.set_ylabel("control_vs [m/s]")
                    if rowid == 0 and colid == 1:
                        axi.plot(summary[episode]['robot'][robot]['control_times'],
                                 summary[episode]['robot'][robot]['control_ws'], marker="o")
                        axi.set_ylabel("control_ws [rad/s]")
                    if rowid == 1 and colid == 0:
                        axi.plot(summary[episode]['robot'][robot]['odom_times'],
                                 summary[episode]['robot'][robot]['odom_vs'])
                        axi.set_ylabel("odom_vs [m/s]")
                        axi.set_xlabel("time [s]")
                    if rowid == 1 and colid == 1:
                        axi.plot(summary[episode]['robot'][robot]['odom_times'],
                                 summary[episode]['robot'][robot]['odom_ws'])
                        axi.set_ylabel("odom_ws [rad/s]")
                        axi.set_xlabel("time [s]")
                    axi.set_xlim([summary[episode]['robot'][robot]['odom_times'][0] - 0.2,
                                  summary[episode]['robot'][robot]['odom_times'][-1] + 0.2])
                plt.show()
                return
            fig, ax = plt.subplots(nrows=robot_total, ncols=2, constrained_layout=True)
            if data_source == 'control':
                fig.suptitle('Control Velocities of Episode ' + str(episode), fontsize=14)
            elif data_source == 'odom':
                fig.suptitle('Odometry Velocities of Episode ' + str(episode), fontsize=14)
            min_times = []
            max_times = []
            for j in range(robot_total):
                min_times.append(summary[episode]['robot'][j]['control_times'][0])
                max_times.append(summary[episode]['robot'][j]['control_times'][-1])
            min_x = np.array(min_times).min()
            max_x = np.array(max_times).max()
            for i, axi in enumerate(ax.flat):
                robot_i = i // 2
                colid = i % 2
                if colid == 0:
                    if data_source == 'control':
                        axi.plot(summary[episode]['robot'][robot_i][x_key], summary[episode]['robot'][robot_i][v_key],
                                 marker="o")
                    else:
                        axi.plot(summary[episode]['robot'][robot_i][x_key], summary[episode]['robot'][robot_i][v_key])
                    axi.set_ylabel("robot: " + str(robot_i))
                if colid == 1:
                    if data_source == 'control':
                        axi.plot(summary[episode]['robot'][robot_i][x_key], summary[episode]['robot'][robot_i][w_key],
                                 marker="o")
                    else:
                        axi.plot(summary[episode]['robot'][robot_i][x_key], summary[episode]['robot'][robot_i][w_key])
                    axi.set_xlabel("time [s]")
                axi.set_xlim([min_x, max_x])
            ax[0][0].set_title("v [m/s]")
            ax[0][1].set_title("w [rad/s]")
            plt.show()
            return
        for ep in range(ep_total):
            fig, ax = plt.subplots(nrows=robot_total, ncols=2, constrained_layout=True)
            if data_source == 'control':
                fig.suptitle('Control Velocities of Episode ' + str(ep), fontsize=14)
            elif data_source == 'odom':
                fig.suptitle('Odometry Velocities of Episode ' + str(ep), fontsize=14)
            min_times = []
            max_times = []
            for j in range(robot_total):
                min_times.append(summary[ep]['robot'][j]['control_times'][0])
                max_times.append(summary[ep]['robot'][j]['control_times'][-1])
            min_x = np.array(min_times).min()
            max_x = np.array(max_times).max()
            for i, axi in enumerate(ax.flat):
                robot_i = i // 2
                colid = i % 2
                if colid == 0:
                    if data_source == 'control':
                        axi.plot(summary[ep]['robot'][robot_i][x_key], summary[ep]['robot'][robot_i][v_key], marker="o")
                    else:
                        axi.plot(summary[ep]['robot'][robot_i][x_key], summary[ep]['robot'][robot_i][v_key])
                    axi.set_ylabel("robot: " + str(robot_i))
                if colid == 1:
                    if data_source == 'control':
                        axi.plot(summary[ep]['robot'][robot_i][x_key], summary[ep]['robot'][robot_i][w_key], marker="o")
                    else:
                        axi.plot(summary[ep]['robot'][robot_i][x_key], summary[ep]['robot'][robot_i][w_key])
                    axi.set_xlabel("time [s]")
                axi.set_xlim([min_x, max_x])
            ax[0][0].set_title("v [m/s]")
            ax[0][1].set_title("w [rad/s]")
        # fig.tight_layout()
        plt.show()

    def cal_indicator(self, summary=None, interval=0.2):
        print ("cal_indicator ...")
        if summary is None:
            summary = self.distill_trajectory(interval=interval)
            summary = self.distill_goal(summary)
        summary = self.cal_vw(summary)
        ep_total = len(summary)
        robot_total = len(summary[0]['robot'])
        test_arrives = [0] * robot_total
        test_collisions = [0] * robot_total
        test_stucks = [0] * robot_total
        test_arrive_times = []
        test_arrive_distances = []
        test_vs = []
        test_ws = []
        test_extra_time = []
        test_extra_dist = []
        for ep in range(ep_total):
            ep_begin_time = summary[ep]['begin_time']
            ep_arrive_times = [0.0] * robot_total
            ep_arrive_distances = [0.0] * robot_total
            ep_vs = []
            ep_ws = []
            ep_extra_time = [0.0] * robot_total
            ep_extra_dist = [0.0] * robot_total
            for i in range(robot_total):
                robot_end_time = summary[ep]['robot'][i]['end_time']
                robot_res = summary[ep]['robot'][i]['result']
                if robot_res == 'arrive':
                    test_arrives[i] += 1
                    ep_arrive_times[i] = robot_end_time - ep_begin_time
                    ep_arrive_distances[i] = summary[ep]['robot'][i]['distance']
                    pose_vs = summary[ep]['robot'][i]['pose_vs']
                    pose_ws = summary[ep]['robot'][i]['pose_ws']
                    # ep_vs.append(pose_vs[:])
                    # ep_ws.append(pose_ws[:])
                    test_vs += pose_vs
                    test_ws += pose_ws
                    min_dist = dist2(summary[ep]['robot'][i]['goal'],
                                     summary[ep]['robot'][i]['poses'][0]) - self.reach_th
                    ep_extra_time[i] = ep_arrive_times[i] - min_dist / self.max_v
                    ep_extra_dist[i] = ep_arrive_distances[i] - min_dist
                    # print "ep_extra_dist ", ep_extra_dist[i]
                    # min_dist = dist2(summary[ep]['robot'][i]['poses'][-1], summary[ep]['robot'][i]['poses'][0])
                    # print ep_arrive_distances[i] - min_dist
                    # print "arrive dist ", dist2(summary[ep]['robot'][i]['goal'], summary[ep]['robot'][i]['poses'][-1])
                elif robot_res == 'collision':
                    # ep_vs.append([])
                    # ep_ws.append([])
                    test_collisions[i] += 1
                elif robot_res == 'stuck':
                    # ep_vs.append([])
                    # ep_ws.append([])
                    test_stucks[i] += 1
            test_arrive_times.append(ep_arrive_times[:])
            test_arrive_distances.append(ep_arrive_distances[:])
            # test_vs.append(ep_vs[:])
            # test_ws.append(ep_ws[:])
            test_extra_time.append(ep_extra_time[:])
            test_extra_dist.append(ep_extra_dist[:])
        test_arrive_times = np.array(test_arrive_times)
        test_arrive_distances = np.array(test_arrive_distances)
        test_vs = np.array(test_vs)
        test_ws = np.array(test_ws)
        test_extra_time = np.array(test_extra_time)
        test_extra_dist = np.array(test_extra_dist)
        print
        "shape:", test_vs.shape
        for i in range(robot_total):
            print
            colorize("Robot: ", 'blue', True) + colorize(str(i), 'green', True)
            print
            colorize("Arrive rate: ", 'blue', True) + "{0} / {1} = {2:.1f}%".format(
                test_arrives[i], ep_total, float(test_arrives[i]) / (test_arrives[i] + test_collisions[i]))
            print
            colorize("Collision rate: ", 'blue', True) + "{0} / {1} = {2:.1f}%".format(
                test_collisions[i], ep_total, float(test_collisions[i]) / (test_arrives[i] + test_collisions[i]))
            print
            colorize("Stuck rate: ", 'blue', True) + "{0} / {1} = {2:.1f}%".format(
                test_stucks[i], ep_total, float(test_stucks[i]) / ep_total * 100)
            print
            colorize("Extra time: ", 'blue', True) + "mean / std : %.4f / %.4f" % (test_extra_time[:, i].mean(),
                                                                                   test_extra_time[:, i].std())
            print
            colorize("Extra distance: ", 'blue', True) + "mean / std : %.4f / %.4f" % (test_extra_dist[:, i].mean(),
                                                                                       test_extra_dist[:, i].std())
            print
            colorize("Time: ", 'blue', True) + "mean / std : %.4f / %.4f" % (test_arrive_times[:, i].mean(),
                                                                             test_arrive_times[:, i].std())
            print
            colorize("Distance: ", 'blue', True) + "mean / std : %.4f / %.4f" % (test_arrive_distances[:, i].mean(),
                                                                                 test_arrive_distances[:, i].std())
            # print colorize("Linear velocity: ", 'blue', True) + "mean / std : %.4f / %.4f" % (
            # test_vs[:,i,:].flatten().mean(), test_vs[:,i,:].flatten().std())
            # print colorize("Angular velocity: ", 'blue', True) + "mean / std : %.4f / %.4f" % (
            # test_ws[:,i,:].flatten().mean(), test_ws[:,i,:].flatten().std())
            print ("============================================")

        test_arrives = np.array(test_arrives)
        test_collisions = np.array(test_collisions)
        test_stucks = np.array(test_stucks)
        print(
        colorize("Average of all robots: ", 'green', True)
        )
        print(
        colorize("Arrive rate: ", 'cyan', True) + "{0} / {1} = {2:.1f}%".format(
            test_arrives.sum(), (test_arrives.sum() + test_collisions.sum()),
            float(test_arrives.sum()) / (test_arrives.sum() + test_collisions.sum()) * 100)
        )
        print(
        colorize("Collision rate: ", 'cyan', True) + "{0} / {1} = {2:.1f}%".format(
            test_collisions.sum(), (test_arrives.sum() + test_collisions.sum()),
            float(test_collisions.sum()) / (test_arrives.sum() + test_collisions.sum()) * 100)
        )
        print(
        colorize("Stuck rate: ", 'cyan', True) + "{0} / {1} = {2:.1f}%".format(
            test_stucks.sum(), ep_total * robot_total, float(test_stucks.sum()) / (ep_total * robot_total) * 100)
        )
        print(
        colorize("Extra time: ", 'cyan', True) + "mean / std : %.4f / %.4f" % (test_extra_time.flatten().mean(),
                                                                               test_extra_time.flatten().std())
        )
        print(
        colorize("Extra distance: ", 'cyan', True) + "mean / std : %.4f / %.4f" % (test_extra_dist.flatten().mean(),
                                                                                   test_extra_dist.flatten().std())
        )
        vs_f = test_vs.flatten()
        print(
        colorize("Linear velocity: ", 'cyan', True) + "mean / std : %.4f / %.4f" % (
            vs_f.mean(), vs_f.std()))
        ws_f = test_ws.flatten()
        print(
        colorize("Angular velocity: ", 'cyan', True) + "mean / std : %.4f / %.4f" % (
            ws_f.flatten().mean(), ws_f.flatten().std()))
        print(
        colorize("Time: ", 'cyan', False) + colorize("mean / std : %.4f / %.4f" % (test_arrive_times.flatten().mean(),
                                                                                   test_arrive_times.flatten().std()),
                                                     'cyan', False))
        print(
        colorize("Distance: ", 'cyan', False) + colorize(
            "mean / std : %.4f / %.4f" % (test_arrive_distances.flatten().mean(),
                                          test_arrive_distances.flatten().std()), 'cyan', False))
        print(
        "============================================")

    # magenta=35,
    # cyan=36,
    # white=37,
    # crimson=38


if __name__ == "__main__":
    # br = BagReader('/home/cgd/catkin_ws/src/drl_nav/data/1584686673.75.bag')
    # # br.draw_trajectory(ep_split=False)
    # br.draw_vw(data_source='control')

    # example: python -m drl.run bag -f ./data/xxx/xxx.bag -p imaget -ep split
    # example2: python envs/utils/bag/bag.py -int 0.5 -p imagepedt -f xx.bag
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--file', '-f')
    parser.add_argument('--plot', '-p', default='index')
    parser.add_argument('--interval', '-int', default='0.2')
    parser.add_argument('--episode', '-ep', default='all')
    parser.add_argument('--robot', '-r', default='0')
    # robot是否需要画每个圈， 见1.png 和 2.png的区别
    parser.add_argument('--robot_every_circle', '-rec', default=False)
    parser.add_argument('--robot_radius', '-radius', default=0.17)
    parser.add_argument('--topic', '-t', default='/image_ped_circle0/episode_res')
    args = parser.parse_args()
    br = BagReader(args)
    print('bag_file: ', args.file)
    if 'image' in args.plot:
        summary = br.distill_img_env_bag(interval=float(args.interval))
        print("len ",len(summary))
        if 'ped' in args.plot:
            br.type = 'image_ped'
        if 't' in args.plot:
            br.draw_trajectory(summary=summary, ep_split=False, episode=args.episode, save_dir=osp.dirname(args.file), robot_every_circle=args.robot_every_circle)
        elif 'vw' in args.plot:
            if 'odom' in args.plot:
                br.draw_vw('odom', summary=summary, episode=args.episode)
            elif 'control' in args.plot or 'cmd_vel' in args.plot:
                br.draw_vw('control', summary=summary, episode=args.episode)
            else:
                br.draw_vw('vw', summary=summary, episode=args.episode, robot=args.robot)
        elif 'data' in args.plot or 'indicator' in args.plot or 'index' in args.plot:
            br.cal_indicator(summary=summary)
    elif args.plot in 'trajectory':
        br.read_tf_tree()
        summary = br.distill_control_vw()
        summary = br.distill_trajectory(interval=float(args.interval), summary=summary)
        summary = br.distill_goal(summary)
        summary = br.distill_world(summary)
        br.draw_trajectory(summary=summary, ep_split=False, episode=args.episode, save_dir=osp.dirname(args.file))
    elif 'vw' in args.plot:
        br.read_tf_tree()
        summary = br.get_episode_time()
        if 'odom' in args.plot:
            summary = br.distill_control_vw(summary=summary)
            summary = br.distill_vw_from_odom(summary=summary)
            br.draw_vw('odom', summary=summary, episode=args.episode)
        elif 'control' in args.plot or 'cmd_vel' in args.plot:
            summary = br.distill_control_vw(summary=summary)
            br.draw_vw('control', summary=summary, episode=args.episode)
        else:
            summary = br.distill_control_vw(summary=summary)
            summary = br.distill_vw_from_odom(summary=summary)
            br.draw_vw('vw', summary=summary, episode=args.episode, robot=args.robot)
    elif 'data' in args.plot or 'indicator' in args.plot or 'index' in args.plot:
        br.read_tf_tree()
        br.cal_indicator(interval=float(args.interval))