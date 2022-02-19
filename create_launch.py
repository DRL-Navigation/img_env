import argparse
import yaml
import os
import rospkg
import sys


def get_pkg_path(pkg_name):
    rospack = rospkg.RosPack()
    return rospack.get_path(pkg_name)


def read_yaml(file: str) -> dict:

    file = open(file, 'r', encoding="utf-8")
    # 读取文件中的所有数据
    file_data = file.read()
    file.close()

    # 指定Loader
    data = yaml.load(file_data, Loader=yaml.FullLoader)
    return data


def construct_launch_file(ins):

    out = """<launch>\n"""
    for node_name, node_num in ins:
        for node_index in range(node_num):
            out += """    <node pkg = "img_env" type = "img_env_node" name = "{}{}" output = "screen" respawn = "true" > </node>\n""".format(node_name, node_index)

    out += """</launch>"""

    return out


def cmd(launch_data, task_name):
    pkg_path = get_pkg_path("img_env")
    tmp_path = pkg_path + "/tmp"
    launch_file_path = "{}/{}.launch".format(tmp_path, task_name)
    # log_file_path = tmp_path + "/" + task_name + ".log"
    if not os.path.exists(tmp_path):
        os.mkdir(tmp_path)
    with open(launch_file_path, "w" ) as f:
        f.write(launch_data)

    # os.system("source ~/.bashrc & nohup roslaunch img_env {} > {} 2>&1 &".format(launch_file_path ,log_file_path))

    print( "You can type following line to roslaunch env nodes: \n" )
    print("roslaunch img_env {}.launch".format(task_name))






if __name__ == "__main__":
    yaml_files = sys.argv[2:]
    task_name = sys.argv[1]
    t = []
    for yaml_file in yaml_files:
        data = read_yaml(yaml_file)
        t.append( ( data['env_name'], data['env_num'] ) )
    launch_data = construct_launch_file(t)
    cmd(launch_data, task_name)
