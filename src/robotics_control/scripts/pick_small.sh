#!/bin/bash
while true
do
    rosservice call /go_to_config "config_name: 'preGraspSmall'"
    rosservice call /go_to_config "config_name: 'graspSmall'"
    rosservice call /go_to_config "config_name: 'preGraspSmall'"
    rosservice call /go_to_config "config_name: 'aboveBoxA'"
done