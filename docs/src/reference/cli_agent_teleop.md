# `srb agent teleop` — Teleoperate Agent

The `srb agent teleop` command enables manual control of robots in Space Robotics Bench simulation environments using keyboard, gamepad, and other input devices.

## Direct Teleoperation

Basic teleop usage requires specifying an environment:

```
srb agent teleop
```

root@ll201:~/ws# srb agent teleop -e
usage: srb agent teleop \[-h\] \[--headless\] \[--hide_ui\] \[--livestream {0,1,2}\] \[--kit_args KIT_ARGS\] -e
{\_aerial,\_aerial_visual,\_ground,\_ground_visual,\_manipulation,\_manipulation_visual,\_orbital,\_orbital_visual,debris_capture,debris_capture_visual,excavation,excavation_visual,locomotion_velocity_tracking
\[--logdir LOGDIR_PATH\] \[--video\] \[--video_length VIDEO_LENGTH\] \[--video_interval VIDEO_INTERVAL\] \[--interface \[{gui,ros} ...\]\]
\[--teleop_device {gamepad,haptic,keyboard,ros,spacemouse} \[{gamepad,haptic,keyboard,ros,spacemouse} ...\]\] \[--pos_sensitivity POS_SENSITIVITY\] \[--rot_sensitivity ROT_SENSITIVITY\] \[--invert_controls\]
\[--algo {dreamer,sb3_a2c,sb3_ars,sb3_crossq,sb3_ddpg,sb3_dqn,sb3_ppo,sb3_ppo_lstm,sb3_qrdqn,sb3_sac,sb3_td3,sb3_tqc,sb3_trpo,sbx_crossq,sbx_ddpg,sbx_dqn,sbx_ppo,sbx_sac,sbx_td3,sbx_tqc,skrl_a2c,skrl_am
\[--model MODEL\]
srb agent teleop: error: argument -e/--env/--task: expected one argument
There was an error running python
root@ll201:~/ws# srb agent teleop --help
usage: srb agent teleop \[-h\] \[--headless\] \[--hide_ui\] \[--livestream {0,1,2}\] \[--kit_args KIT_ARGS\] -e
{\_aerial,\_aerial_visual,\_ground,\_ground_visual,\_manipulation,\_manipulation_visual,\_orbital,\_orbital_visual,debris_capture,debris_capture_visual,excavation,excavation_visual,locomotion_velocity_tracking
\[--logdir LOGDIR_PATH\] \[--video\] \[--video_length VIDEO_LENGTH\] \[--video_interval VIDEO_INTERVAL\] \[--interface \[{gui,ros} ...\]\]
\[--teleop_device {gamepad,haptic,keyboard,ros,spacemouse} \[{gamepad,haptic,keyboard,ros,spacemouse} ...\]\] \[--pos_sensitivity POS_SENSITIVITY\] \[--rot_sensitivity ROT_SENSITIVITY\] \[--invert_controls\]
\[--algo {dreamer,sb3_a2c,sb3_ars,sb3_crossq,sb3_ddpg,sb3_dqn,sb3_ppo,sb3_ppo_lstm,sb3_qrdqn,sb3_sac,sb3_td3,sb3_tqc,sb3_trpo,sbx_crossq,sbx_ddpg,sbx_dqn,sbx_ppo,sbx_sac,sbx_td3,sbx_tqc,skrl_a2c,skrl_am
\[--model MODEL\]

options:
-h, --help            show this help message and exit

Launcher:
--headless            Run the simulation without display output (default: False)
--hide_ui             Disable most of the Isaac Sim UI and set it to fullscreen (default: False)
--livestream {0,1,2}  Force enable livestreaming. Mapping corresponds to that for the `LIVESTREAM` environment variable (0: Disabled, 1: Native, 2: WebRTC) (default: -1)
--kit_args KIT_ARGS   CLI args for the Omniverse Kit as a string separated by a space delimiter (e.g., '--ext-folder=/path/to/ext1 --ext-folder=/path/to/ext2') (default: )

Environment:
-e {\_aerial,\_aerial_visual,\_ground,\_ground_visual,\_manipulation,\_manipulation_visual,\_orbital,\_orbital_visual,debris_capture,debris_capture_visual,excavation,excavation_visual,locomotion_velocity_tracking,locomotion_velocity_tracking_visual,mobile_debris_capture,mobile_debris_capture_visual,orbital_evasion,orbital_evasion_visual,peg_in_hole_assembly,peg_in_hole_assembly_multi,peg_in_hole_assembly_multi_visual,peg_in_hole_assembly_visual,sample_collection,sample_collection_multi,sample_collection_multi_visual,sample_collection_visual,solar_panel_assembly,solar_panel_assembly_visual}, --env {\_aerial,\_aerial_visual,\_ground,\_ground_visual,\_manipulation,\_manipulation_visual,\_orbital,\_orbital_visual,debris_capture,debris_capture_visual,excavation,excavation_visual,locomotion_velocity_tracking,locomotion_velocity_tracking_visual,mobile_debris_capture,mobile_debris_capture_visual,orbital_evasion,orbital_evasion_visual,peg_in_hole_assembly,peg_in_hole_assembly_multi,peg_in_hole_assembly_multi_visual,peg_in_hole_assembly_visual,sample_collection,sample_collection_multi,sample_collection_multi_visual,sample_collection_visual,solar_panel_assembly,solar_panel_assembly_visual}, --task {\_aerial,\_aerial_visual,\_ground,\_ground_visual,\_manipulation,\_manipulation_visual,\_orbital,\_orbital_visual,debris_capture,debris_capture_visual,excavation,excavation_visual,locomotion_velocity_tracking,locomotion_velocity_tracking_visual,mobile_debris_capture,mobile_debris_capture_visual,orbital_evasion,orbital_evasion_visual,peg_in_hole_assembly,peg_in_hole_assembly_multi,peg_in_hole_assembly_multi_visual,peg_in_hole_assembly_visual,sample_collection,sample_collection_multi,sample_collection_multi_visual,sample_collection_visual,solar_panel_assembly,solar_panel_assembly_visual}
Name of the environment to select (default: None)

Logging:
--logdir LOGDIR_PATH, --logs LOGDIR_PATH
Path to the root directory for storing logs (default: /root/ws/logs)

Video Recording:
--video               Record videos (default: False)
--video_length VIDEO_LENGTH
Length of the recorded video (in steps) (default: 1000)
--video_interval VIDEO_INTERVAL
Interval between video recordings (in steps) (default: 10000)

Interface:
--interface \[{gui,ros} ...\]
Sequence of interfaces to enable (default: \[\])

Teleop:
--teleop_device {gamepad,haptic,keyboard,ros,spacemouse} \[{gamepad,haptic,keyboard,ros,spacemouse} ...\]
Device for interacting with environment (default: \['keyboard'\])
--pos_sensitivity POS_SENSITIVITY
Sensitivity factor for translation (default: 10.0)
--rot_sensitivity ROT_SENSITIVITY
Sensitivity factor for rotation (default: 40.0)
--invert_controls, --invert
Flag to invert the controls for translation (default: False)

Teleop Policy:
--algo {dreamer,sb3_a2c,sb3_ars,sb3_crossq,sb3_ddpg,sb3_dqn,sb3_ppo,sb3_ppo_lstm,sb3_qrdqn,sb3_sac,sb3_td3,sb3_tqc,sb3_trpo,sbx_crossq,sbx_ddpg,sbx_dqn,sbx_ppo,sbx_sac,sbx_td3,sbx_tqc,skrl_a2c,skrl_amp,skrl_cem,skrl_ddpg,skrl_ddqn,skrl_dqn,skrl_ippo,skrl_mappo,skrl_ppo,skrl_ppo_rnn,skrl_rpo,skrl_sac,skrl_td3,skrl_trpo}
Name of the algorithm (default: None)
--model MODEL         Path to the model checkpoint (default: None)

## Teleoperation via Policy
