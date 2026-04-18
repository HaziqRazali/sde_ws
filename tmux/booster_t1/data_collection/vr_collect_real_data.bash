#!/bin/sh

SESSION_NAME=tmux_run
ROBOT_TYPE=booster_t1
COLLECT_DATA=true
VR_DEVICE=metaquest3
SIM_OR_REAL=real
IP_ADDRESS=192.168.10.13
SIM_ENV_NAME=default
USE_SIM_TIME=false


tmux has-session -t $SESSION_NAME
if [ $? != 0 ]; then
  # create a new session
  tmux new-session -s $SESSION_NAME -n $SESSION_NAME -d

  # use mouse in Ubuntu / tmux 2.1
  tmux set -g mouse on

  # Highlight active window
  tmux set-window-option -g window-status-current-bg green

  # history limit
  tmux set -g history-limit 10000

  # Set status bar
  tmux set -g status-bg black
  tmux set -g status-fg white 

  # to enable mouse copy/paste to system buffer
  # to copy text, go to the any window and press this sequence
  #     C-b [
  # now you are in VI mode, use the arrow keys and move to the start of text to copy
  # To select the text, press the key 'v'
  # Continue moving using the arrow keys to highlight the text to copy
  # To copy the text, press the 'y' key
  # Now your highlighted text is in system clipboard, can just use ctrl-V to paste into text editor
  # if you want to paste to another tmux buffer, press this sequence
  #     C-b ]
  tmux setw -g mode-keys vi
  #https://unix.stackexchange.com/questions/131011/use-system-clipboard-in-vi-copy-mode-in-tmux
  tmux bind-key -T copy-mode-vi 'v' send-keys -X begin-selection
  tmux bind-key -T copy-mode-vi 'y' send-keys -X copy-pipe-and-cancel 'xclip -in -selection clipboard'

  # ----- running with gdb -----
  #ros2 run --prefix 'gdb -ex run --args' <pkg> <node> --all-other-launch arguments
  #ros2 launch --launch-prefix 'xterm -e gdb -ex run --args' test_package start.launch.py robot_type:='$ROBOT_TYPE'

  # ========== restart the ros2 daemon ==========
  tmux send-keys -t $SESSION_NAME 'ros2 daemon stop && ros2 daemon start' C-m
  tmux select-layout -t $SESSION_NAME tiled

  tmux split-window -v -t $SESSION_NAME
  tmux send-keys -t $SESSION_NAME 'tmux kill-server'
  tmux select-layout -t $SESSION_NAME tiled

  # ========== sde ==========
  # VR tf on rviz visualization and metaquest3 teleoperation (with data collection)
  tmux new-window -n sde
  tmux send-keys -t $SESSION_NAME:sde 'ros2 launch hardware_interfaces hardware_interface.launch.py client_ip:=' $IP_ADDRESS ' config_name:=booster_t1_hardware_interface.yaml'
  tmux select-layout -t $SESSION_NAME:sde tiled

  tmux split-window -v -t $SESSION_NAME:sde
  tmux send-keys -t $SESSION_NAME:sde 'ros2 launch casadi_optimal_control_ik robot_ik_solver.launch.py robot_type:=' $ROBOT_TYPE ' sim_or_real:=' $SIM_OR_REAL ' config_file:=ik_solver_config.yaml enable_tf:=true enable_static_world_tf:=true robot_base_link_name:=Trunk use_sim_time:=' $USE_SIM_TIME
  tmux select-layout -t $SESSION_NAME:sde tiled
  
  tmux split-window -v -t $SESSION_NAME:sde
  tmux send-keys -t $SESSION_NAME:sde 'ros2 launch teleoperation_interface teleoperation_interface.launch.py vr_device:=' $VR_DEVICE ' robot_type:=' $ROBOT_TYPE ' sim_or_real:=' $SIM_OR_REAL ' collect_data:=' $COLLECT_DATA ' teleop_config_file:=teleoperation_interface_config.yaml task_config_file:=task_labelling_config.yaml enable_rviz_gui:=true use_sim_time:=' $USE_SIM_TIME C-m
  tmux select-layout -t $SESSION_NAME:sde tiled

  tmux split-window -v -t $SESSION_NAME:sde
  tmux send-keys -t $SESSION_NAME:sde 'echo Please check that this bash script IP ADDRESS is set correctly - your system network interface IP for connection to robot' C-m
  tmux send-keys -t $SESSION_NAME:sde 'echo Please set robot to custom mode before running hardware_interface & ik_solver! (mp - md - mc)' C-m
  tmux send-keys -t $SESSION_NAME:sde 'cd /opt/ros/humble/lib/booster_robotics_sdk && ./b1_loco_example_client ' $IP_ADDRESS C-m # please put robot into mc mode (custom) before running hardware_interfaces.
  tmux select-layout -t $SESSION_NAME:sde tiled

  tmux split-window -v -t $SESSION_NAME:sde
  tmux send-keys -t $SESSION_NAME:sde 'echo ros2 launch ros2bag_server ros2bag_server.launch.py robot_type:=' $ROBOT_TYPE ' config_name:=data_collection_config.yaml path_to_output_dir:=/root/ output_dir:=bag/ is_run_on_robot:=false is_mcap:=false is_override_qos:=false' C-m
  tmux send-keys -t $SESSION_NAME:sde 'echo please run the above in robot PC and check that the variable: is_run_on_robot:=true AND path_to_output_dir:=/home/$USER' C-m
  tmux select-layout -t $SESSION_NAME:sde tiled


fi

if [ -z "$TMUX" ]; then
  tmux attach -t $SESSION_NAME
else
  tmux switch-client -t $SESSION_NAME
fi
