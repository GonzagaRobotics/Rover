#!/bin/bash

# Use inside the first level of the Rover directory. No arguments

rsync -r --delete --delete-excluded \
  --exclude=build \
  --exclude=install \
  --exclude=log \
  ../Rover jetson@10.0.0.1:~/