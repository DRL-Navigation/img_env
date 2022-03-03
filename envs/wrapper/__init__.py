from envs.wrapper.base import *
from envs.wrapper.filter_states import *


wrapper_dict = {
    "StatePedVectorWrapper": StatePedVectorWrapper,
    "VelActionWrapper": VelActionWrapper,
    "StateBatchWrapper": StateBatchWrapper,
    "SensorsPaperRewardWrapper": SensorsPaperRewardWrapper,
    "NeverStopWrapper": NeverStopWrapper,
    "ObsStateTmp": ObsStateTmp,
    "TimeLimitWrapper": TimeLimitWrapper,
    "MultiRobotCleanWrapper": MultiRobotCleanWrapper,
    "InfoLogWrapper": InfoLogWrapper,
    'TestEpisodeWrapper': TestEpisodeWrapper,
}



"""
wrapper
from XXX import A,B,C
# read config file
# Sequentially reading wrapper class and warper the env one by one.
env = ImageEnv()
env = A(env)
。。。

# reward warpper
# batch wrapper

"""
