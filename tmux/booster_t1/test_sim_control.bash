#!/bin/sh

SESSION_NAME=tmux_run
ROBOT_TYPE=booster_t1
COLLECT_DATA=false
VR_DEVICE=disable
SIM_OR_REAL=sim
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
  # Sim tf on rviz visualization and interactive marker control (without data collection)
  tmux new-window -n sde
  tmux send-keys -t $SESSION_NAME:sde 'ros2 launch sde_simulation sim_main_genesis.launch.py robot_type:=' $ROBOT_TYPE ' sim_env_name:=' $SIM_ENV_NAME ' enable_gui:=true num_ROS2_threads:=8 enable_sim_tf:=false use_sim_time:=' $USE_SIM_TIME C-m
  tmux select-layout -t $SESSION_NAME:sde tiled

  tmux split-window -v -t $SESSION_NAME:sde
  tmux send-keys -t $SESSION_NAME:sde 'sleep 15; ros2 launch casadi_optimal_control_ik robot_ik_solver.launch.py robot_type:=' $ROBOT_TYPE ' sim_or_real:=' $SIM_OR_REAL ' config_file:=ik_solver_config.yaml enable_tf:=true enable_static_world_tf:=true robot_base_link_name:=Trunk use_sim_time:=' $USE_SIM_TIME C-m
  tmux select-layout -t $SESSION_NAME:sde tiled
  
  tmux split-window -v -t $SESSION_NAME:sde
  tmux send-keys -t $SESSION_NAME:sde 'sleep 20; ros2 launch teleoperation_interface teleoperation_interface.launch.py vr_device:=' $VR_DEVICE ' robot_type:=' $ROBOT_TYPE ' sim_or_real:=' $SIM_OR_REAL ' collect_data:=' $COLLECT_DATA ' teleop_config_file:=teleoperation_interface_config.yaml task_config_file:=task_labelling_config.yaml enable_rviz_gui:=false use_sim_time:=' $USE_SIM_TIME C-m
  tmux select-layout -t $SESSION_NAME:sde tiled

  tmux split-window -v -t $SESSION_NAME:sde
  tmux select-layout -t $SESSION_NAME:sde tiled



fi

if [ -z "$TMUX" ]; then
  tmux attach -t $SESSION_NAME
else
  tmux switch-client -t $SESSION_NAME
fi
