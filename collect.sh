#!/bin/bash

PROGRAM="./build/guidestereo"
RUN_TIME=60
PAUSE_TIME=90

trap "echo 'User interrupted, exiting'; exit 0" INT

while true; do
    HOST_TIME=$(date +"%Y%m%d_%H%M%S")
    ARG_PATH="./capture/sequence1/${HOST_TIME}"

    echo "Starting program..."
    $PROGRAM "1" "$ARG_PATH" < /dev/tty &
    PID=$!

    sleep $RUN_TIME

    echo "Stopping program (SIGTERM)..."
    kill -TERM $PID
    wait $PID

    echo "Program exited."
    echo "Sleeping for $PAUSE_TIME seconds..."
    sleep $PAUSE_TIME
done


