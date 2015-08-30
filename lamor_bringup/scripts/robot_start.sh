#!/bin/bash

SESSION=$USER

tmux -2 new-session -d -s $SESSION
# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'core'
tmux new-window -t $SESSION:2 -n 'robot'
tmux new-window -t $SESSION:3 -n 'cameras'
tmux new-window -t $SESSION:4 -n 'ui'
tmux new-window -t $SESSION:5 -n 'navigation'
tmux new-window -t $SESSION:6 -n 'ppl_perception'
#tmux new-window -t $SESSION:7 -n 'scheduler'
#tmux new-window -t $SESSION:8 -n 'control'
#New windows:
tmux new-window -t $SESSION:7 -n 'mary'
tmux new-window -t $SESSION:8 -n 'facts'
tmux new-window -t $SESSION:9 -n 'weather'
tmux new-window -t $SESSION:10 -n 'random_walk' 
tmux new-window -t $SESSION:11 -n 'controller'



tmux select-window -t $SESSION:0
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "roscore" C-m
tmux resize-pane -U 30
tmux select-pane -t 1
tmux send-keys "htop" C-m

tmux select-window -t $SESSION:1
tmux send-keys "DISPLAY=:0 roslaunch mongodb_store mongodb_store.launch db_path:=$HOME/mongodb_lamor"

tmux select-window -t $SESSION:2
tmux send-keys "DISPLAY=:0 roslaunch strands_bringup strands_robot.launch with_mux:=false with_magnetic_barrier:=false"

tmux select-window -t $SESSION:3
tmux send-keys "DISPLAY=:0 roslaunch strands_bringup strands_cameras.launch head_camera:=true head_ip:=$HEAD_PC head_user:=lamor chest_camera:=true chest_ip:=$CHEST_PC chest_user:=lamor"

tmux select-window -t $SESSION:4
tmux send-keys "HOST_IP=192.168.0.100 DISPLAY=:0 roslaunch strands_ui strands_ui.launch mary_machine:=$HEAD_PC mary_machine_user:=lamor"

tmux select-window -t $SESSION:5
tmux send-keys "DISPLAY=:0 roslaunch lamor_bringup lamor_navigation.launch chest_xtion_machine:=$CHEST_PC chest_xtion_user:=$USER head_xtion_machine:=$HEAD_PC head_xtion_user:=$USER topo_nav_machine:=$CHEST_PC topo_nav_user:=$USER map:=\$(rospack find lamor_bringup)/resources/betty_bl.yaml topological_map:=betty"

tmux select-window -t $SESSION:6
tmux send-keys "DISPLAY=:0 roslaunch perception_people_launch people_tracker_robot.launch machine:=$HEAD_PC user:=lamor"

tmux select-window -t $SESSION:7
tmux send-keys "DISPLAY=:0 roslaunch mary_tts ros_mary.launch"

tmux select-window -t $SESSION:8
tmux send-keys "DISPLAY=:0 rosrun facts tellFacts.py"

tmux select-window -t $SESSION:9
tmux send-keys "DISPLAY=:0 rosrun weather tell_weather.py"

tmux select-window -t $SESSION:10
tmux send-keys "DISPLAY=:0 rosrun random_walker random_walker.py"

tmux select-window -t $SESSION:11
tmux send-keys "DISPLAY=:0 roslaunch controller controller_robot.launch"


# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION

tmux setw -g mode-mouse off
