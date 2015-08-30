#!/bin/bash

SESSION=$USER

tmux -2 new-session -d -s $SESSION
# Setup a window for tailing log files
tmux new-window -t $SESSION:100 -n 'ppl_perception'
tmux new-window -t $SESSION:101 -n 'mary'
tmux new-window -t $SESSION:102 -n 'controller'
tmux new-window -t $SESSION:103 -n 'facts'
tmux new-window -t $SESSION:104 -n 'weather'
tmux new-window -t $SESSION:105 -n 'random_walk'

tmux select-window -t $SESSION:100
tmux send-keys "DISPLAY=:0 roslaunch perception_people_launch people_tracker_robot.launch machine:=$HEAD_PC user:=lamor"

tmux select-window -t $SESSION:101
tmux send-keys "DISPLAY=:0 roslaunch mary_tts ros_mary.launch"

tmux select-window -t $SESSION:102
tmux send-keys "DISPLAY=:0 roslaunch controller controller_robot.launch"

tmux select-window -t $SESSION:103
tmux send-keys "DISPLAY=:0 rosrun facts tellFacts.py"

tmux select-window -t $SESSION:104
tmux send-keys "DISPLAY=:0 rosrun weather tell_weather.py"

tmux select-window -t $SESSION:105
tmux send-keys "DISPLAY=:0 rosrun random_walker random_walker.py"

# Set default window
tmux select-window -t $SESSION:100

# Attach to session
tmux -2 attach-session -t $SESSION

tmux setw -g mode-mouse off
