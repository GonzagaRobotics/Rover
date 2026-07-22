#!/bin/bash

case "$1" in
    "" )
        echo "Available commands:
    rm-all-containers   Attempts to remove all containers"
        ;;
    "rm-all-containers" )
        TO_RM=$(sudo docker container ls -aq)

        if [[ "$TO_RM" != "" ]]; then
            sudo docker container rm $TO_RM
        fi
        ;;
esac