import random
import yaml
import time
from envs import make_env, read_yaml


class RandomPolicy4Nav:
    def __init__(self, n, v_range=(0, 0.6), w_range=(-0.9, 0.9)):
        self.n = n
        self.v_range = v_range
        self.w_range = w_range

    def gen_action(self):
        out = []

        for i in range(self.n):
            out.append( (random.uniform(*self.v_range), random.uniform(*self.w_range), 0 ) )

        return out


if __name__ == "__main__":
    import sys
    tmp = len(sys.argv)
    if tmp == 2:
        cfg = read_yaml(sys.argv[1])
    else:
        cfg = read_yaml('envs/cfg/circle.yaml')
    print(cfg)
    env = make_env(cfg)
    # env2 = make_env(cfg)
    # time.sleep(1)
    random_policy = RandomPolicy4Nav(env.robot_total)
    # test continuous action
    for i in range(100):
        print(11111111111111111,flush=True)
        env.reset()
        print(22222222222222222,flush=True)
        # env2.reset()
        while 1:
            env.step(random_policy.gen_action())
            # env2.step(random_policy.gen_action())

    # for i in range(50):
    #     print(env.step(random_policy.gen_action()))

    # test action
