import gym

from envs.state import ImageState


class ObsStateTmp(gym.ObservationWrapper):
    def __init__(self, env, cfg):
        super(ObsStateTmp, self).__init__(env)

    def observation(self, states: ImageState):

        return [states.sensor_maps, states.vector_states, states.ped_maps]


class ObsLaserStateTmp(gym.ObservationWrapper):
    def __init__(self, env, cfg):
        super(ObsLaserStateTmp, self).__init__(env)

    def observation(self, states: ImageState):
        return [states.lasers, states.vector_states, states.ped_maps]
