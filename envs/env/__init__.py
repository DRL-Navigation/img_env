try:
    from envs.env.yaml_env import ImageEnv
except Exception as e:
    print(str(e))
    print("envs/env/__init__.py: from envs.env import ImageEnv \n "
          "import ERROR, may be you are not `catkin_make image_env`")

from envs.env.real_env import RealEnv
from envs.env.gazebo_env import GazeboEnv