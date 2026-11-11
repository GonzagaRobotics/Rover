#!/bin/bash

# Use inside the first level of the Rover directory. No arguments

rsync -r --delete --delete-excluded \
  --exclude=build \
  --exclude=install \
  --exclude=log \
  --exclude=.venv \
  ../Rover jetson@jetson.local:~/