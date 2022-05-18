#!/bin/sh
pids=$(ps -ef | grep my_process | grep -v "grep" | awk '{print $2}')
kill -9  ${pids}