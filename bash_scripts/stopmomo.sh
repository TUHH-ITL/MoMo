#!/bin/bash

# Define the remote user and host
REMOTE_USER=momo
REMOTE_HOST=192.168.1.1    	# change ip address (of NUC) if different

# Define the shutdown command. Password for the NUC is 1234 (change if different).
SHUTDOWN_CMD="echo '1234' | sudo -S shutdown -h now"

# Connect to the remote host and execute the shutdown command
ssh ${REMOTE_USER}@${REMOTE_HOST} "${SHUTDOWN_CMD}"

poweroff

