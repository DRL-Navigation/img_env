## RECORD INITIAL POSE
fix the initial pose of agent while doing the comparative experiment.

1. param in cfg file: `init_pose_bag_episodes` and `init_pose_bag_name`
```
python envs/utils/bag/save_init_pose.py
```
make sure `cfg_type` in cfg file is "yaml"

2. set `test` with True and `cfg_type` with "bag" in cfg file
```buildoutcfg
python env_test.py
```

## DRAW TRAJECTORY PNG

1. add `"BagRecordWrapper"` in the **first** `wrapper` param of yaml file
```buildoutcfg
wrapper: [
  'BagRecordWrapper',
  ...
]
```   
in yaml file, you can set bag recording episodes and bag file name.
2. run your env, record bag.
```buildoutcfg
python env_test.py
```
record_topic: "/" + cfg['env_name'] + str(cfg['node_id']) + "/episode_res"
3. draw trajectory by generated bag file, example see 2.png
```buildoutcfg
python envs/utils/bag/bag.py -int 0.5 -p imagepedt -f XX.bag
```
if you need draw robot each step, like 1.png
```buildoutcfg
python envs/utils/bag/bag.py -int 0.5 -p imagepedt -f XX.bag -rec true
```