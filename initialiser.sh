#!/bin/sh
cd /home/joshua
while true; do
    python ~/main.py &> '~/logs/$(date).log'
    sleep 1
done