#!/bin/bash
rosservice call /ur_hardware_interface/set_speed_slider "speed_slider_fraction: 0.1" 
rosservice call /ur_hardware_interface/dashboard/pause "{}"