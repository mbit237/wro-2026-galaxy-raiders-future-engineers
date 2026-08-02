#!/bin/sh
cd /home/joshua

ln -sf main_sleep.py main.py
pkill -f 'python main.py'