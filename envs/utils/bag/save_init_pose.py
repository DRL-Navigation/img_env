import yaml
import sys
sys.path.append(sys.path[0]+"/../../../")

from envs.env import ImageEnv


def read_yaml(file: str) -> dict:

    file = open(file, 'r', encoding="utf-8")
    # 读取文件中的所有数据
    file_data = file.read()
    file.close()

    # 指定Loader
    data = yaml.load(file_data, Loader=yaml.FullLoader)
    return data


if __name__ == "__main__":
    f = read_yaml("envs/cfg/random.yaml")
    env = ImageEnv(f)
    env.save_envs_bag(f['init_pose_bag_episodes'], f['init_pose_bag_name'])
