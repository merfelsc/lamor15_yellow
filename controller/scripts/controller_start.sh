#!/bin/bash

SESSION=$USER

tmux -2 new-session -d -s $SESSION
# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'mary'
tmux new-window -t $SESSION:1 -n 'controller'


tmux select-window -t $SESSION:0
tmux send-keys "DISPLAY=:0 roslaunch mary_tts ros_mary.launch"

tmux select-window -t $SESSION:1
tmux send-keys "DISPLAY=:0 roslaunch controller controller_robot.launch"

# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION

tmux setw -g mode-mouse off
