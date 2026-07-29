#!/bin/bash

case "$1" in
    "" )
        echo "Available commands (needs sudo):
    rm-all-containers   Attempts to remove all containers
    rover               Runs the rover container in its recommended configuration"
        ;;
    "rm-all-containers" )
        TO_RM=$(docker container ls -aq)

        if [[ "$TO_RM" != "" ]]; then
            docker container rm $TO_RM
        fi
        ;;
    "rover" )
        docker container run --rm --network=host --ipc=host --mount type=bind,src=../Rover,dst=/Rover -it rover
        ;;
esac