# OpenLoongCtrl_ROS2
OpenLoong humanoid robot for ROS2

##Installation
python scripts/train.py --task=humanoid_ppo --run_name v1 --headless --num_envs 4096
python scripts/play.py --task=humanoid_ppo --run_name v1
python scripts/sim2sim.py --load_model /path/to/logs/XBot_ppo/exported/policies/policy_1.pt
python scripts/sim2sim.py --load_model /path/to/logs/XBot_ppo/exported/policies/policy_example.pt



## Thanks

1.https://atomgit.com/openloong

2.https://atomgit.com/openloong/OpenLoongROS/tree/master/azureloong_control
