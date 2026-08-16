#!/bin/sh
cd /home/joshua
while true; do
    python main.py > "logs/$(date).log" 2>&1
    python stop.py
    sleep 1
done