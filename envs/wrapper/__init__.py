from envs.wrapper.base import *
from envs.wrapper.filter_states import *
from envs.wrapper.test_wrapper import *
from envs.wrapper.evaluation_wrapper.utils import *
from envs.wrapper.evaluation_wrapper.PedTrajectoryDatasetWrapper import *
from envs.wrapper.evaluation_wrapper.RealTestRecoderWrapper import *
from envs.wrapper.evaluation_wrapper.BarnDataSetWrapper import *
from envs.wrapper.evaluation_wrapper.TestEpisodeWrapper import *


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
    "BagRecordWrapper": BagRecordWrapper,
    'TestEpisodeWrapper': TestEpisodeWrapper,
    'ObsLaserStateTmp': ObsLaserStateTmp,
    'BarnDataSetWrapper': BarnDataSetWrapper,
    'TimeControlWrapper': TimeControlWrapper,
    'RealTestRecoderWrapper': RealTestRecoderWrapper,
    'PedTrajectoryDatasetWrapper': PedTrajectoryDatasetWrapper,
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
