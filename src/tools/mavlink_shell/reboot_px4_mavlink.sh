#!/bin/bash

(echo "reboot"; sleep 1) | ~/safm_ws/src/tools/mavlink_shell/mavlink_shell.py -b 921600 /dev/ttyACM0
# Wait for port to disappear
while [ -e /dev/ttyACM0 ]; do sleep 0.2; done

sleep 1

# Wait for port to come back
while [ ! -e /dev/ttyACM0 ]; do sleep 0.2; done
