import numpy as np


class ImageState():
    def __init__(self,
                 vector_states,
                 sensor_maps,
                 is_collisions,
                 is_arrives,
                 lasers,
                 ped_vector_states,
                 ped_maps,
                 step_ds,
                 ped_min_dists,

                 ):
        assert len(vector_states) == len(sensor_maps) == len(is_collisions) == len(is_arrives) == len(lasers) \
                == len(ped_vector_states) == len(ped_maps) == len(step_ds) == len(ped_min_dists)

        self.vector_states = vector_states
        self.sensor_maps = sensor_maps
        self.is_collisions = is_collisions
        self.is_arrives = is_arrives
        self.lasers = lasers
        self.ped_vector_states = ped_vector_states
        self.ped_maps = ped_maps
        self.ped_min_dists = ped_min_dists
        self.step_ds = step_ds

        # self.step_ds = np.zeros_like(len(self.vector_states), dtype=np.float)

    def __str__(self):
        return """Image State Info:
        vector_states: {}
        sensor_maps: {}
        is_collisions: {}
        is_arrives: {}
        lasers: {}
        ped_vector_states: {} 
        ped_maps: {}
        ped_min_dists: {}
        step distance: {}
        """.format(self.vector_states,
                   self.sensor_maps,
                   self.is_collisions,
                   self.is_arrives,
                   self.lasers,
                   self.ped_vector_states,
                   self.ped_maps,
                   self.ped_min_dists,
                   self.step_ds)
        # print("Image State Info:")
        # print("vector_states: ", self.vector_states)
        # print("sensor_maps: ", self.sensor_maps)
        # print("is_collisions: ", self.is_collisions)
        # print("is_arrives: ", self.is_arrives)
        # print("lasers: ", self.lasers)
        # print("ped_vector_states: ", self.ped_vector_states)
        # print("ped_maps: ", self.ped_maps)
        # print("ped_min_dists:", self.ped_min_dists)
        # print("step distance: ", self.step_ds)

    # def update(self,vector_states,
    #              sensor_maps,
    #              is_collisions,
    #              is_arrives,
    #              lasers,
    #              ped_vector_states,
    #              ped_maps,
    #              distances,
    #              ped_min_dists,):
    #     self.vector_states = vector_states
    #     self.sensor_maps = sensor_maps
    #     self.is_collisions = is_collisions
    #     self.is_arrives = is_arrives
    #     self.lasers = lasers
    #     self.ped_vector_states = ped_vector_states
    #     self.ped_maps = ped_maps
    #     self.ped_min_dists = ped_min_dists
    #     ######################################
    #     self.step_ds = self.distances - distances
    #     self.distances = distances

    # def __getitem__(self, index):




    def __len__(self):
        return len(self.vector_states)