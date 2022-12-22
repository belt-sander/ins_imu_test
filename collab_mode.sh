#!/bin/bash

rosservice call /xarm/clear_err
rosservice call /xarm/set_mode 2
rosservice call /xarm/set_state 0
