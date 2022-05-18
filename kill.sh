#!/bin/sh
pids=$(ps -ux | grep ros | grep -v "grep" | awk '{print $2}')
kill -9  ${pids}