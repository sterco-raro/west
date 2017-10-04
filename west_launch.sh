#!/bin/bash

# launch rosbridge_server and west_tools_node nodes and redirect
# their standard output and error to a file
[[ ! -d logs ]] && mkdir logs

rm -rf logs/*

stdbuf -oL 												\
  roslaunch rosbridge_server rosbridge_websocket.launch \
  1>logs/rosbridge_websocket.out 						\
  2>logs/rosbridge_websocket.err &

stdbuf -oL 												\
  rosrun west_tools west_tools_node.py					\
  1>logs/west_tools_node.out 							\
  2>logs/west_tools_node.err &
