#!/bin/sh
cd /home/joshua

ln -sf ~/main_obstacle.py ~/main.py
pkill -f 'python main.py'