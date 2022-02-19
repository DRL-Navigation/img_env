import yaml

from typing import Union

from envs.env import ImageEnv
from envs.wrapper import wrapper_dict


def read_yaml(file: str) -> dict:

    file = open(file, 'r', encoding="utf-8")
    # 读取文件中的所有数据
    file_data = file.read()
    file.close()

    # 指定Loader
    data = yaml.load(file_data, Loader=yaml.FullLoader)
    return data


def make_env(cfg: Union[dict, str]):
    if isinstance(cfg, str):
        cfg = read_yaml(cfg)
    env = ImageEnv(cfg)
    for env_string in cfg['wrapper']:
        env = wrapper_dict[env_string](env, cfg)
    cfg['node_id'] += 1
    return env

__all__ = []