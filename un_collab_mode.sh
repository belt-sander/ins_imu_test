#!/bin/bash

rosservice call /xarm/clear_err
rosservice call /xarm/set_mode 1
rosservice call /xarm/set_state 0
