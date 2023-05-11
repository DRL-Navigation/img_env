## 静态gazebobenchmark对应的wrapper：
    BarnDataSetWrapper.py


## 动态行人数据集对应的wrapper：
    wrapper：PedTrajectoryDatasetWrapper.py
    yaml参数：drlnav_env/envs/cfg/ped_dataset_cfg/test_eth.yaml

一共分为5个数据集
```
ETH：EWAP的数据集包括两个sequence：eth和hotel
UCY的数据集包括三个sequence：univ, zara01，zara02
参考链接：
https://blog.csdn.net/T_C_Ko/article/details/121961696
数据集链接：
https://www.dropbox.com/s/8n02xqv3l9q18r1/datasets.zip?dl=0&file_subpath=%2Fdatasets
目前使用的数据集csv直接使用SocNavBench整理好的：
https://github.com/CMU-TBD/SocNavBench/tree/master/agents/humans/datasets

数据集都是室外的，行人行走总体上还是规律的，大部分走成一条直线，不是随机乱走。
```

重要参数以下：
```
ped_traj_dataset: 数据集csv文件位置
offset: 行人起点的偏移量
ped_dataset_worlds: 填入形式如[[a,b]]，代表数据集里第a个行人至第b个行人
control_hz: 填入0.2的整数倍，目前默认0.4s

####
另外机器人的起终点 终点决定了交互模式：跟随人群，面对人群还是横着穿过人群
```


## 真实环境对应的wrapper：
    RealTestRecoderWrapper.py

## 2D仿真对应的wrapper：
    TestEpisodeWrapper.py

