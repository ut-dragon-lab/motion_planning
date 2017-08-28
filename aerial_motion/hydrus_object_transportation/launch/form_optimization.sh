#!/bin/bash

rosnode kill /hydrusx/joint_state_publisher

roslaunch hydrus_object_transportation form_optimization.launch
