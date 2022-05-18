#!/bin/bash
ps -ux|grep ros |grep -v ros |cut -c 9-16 |xargs kill -9 