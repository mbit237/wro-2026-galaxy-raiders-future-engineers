#!/bin/sh
cd /home/joshua

ln -sf ~/main_open.py ~/main.py
pkill -f 'python main.py'