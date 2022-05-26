#!/bin/sh
sleep 5
/home/hsy/桌面/dang/build/run
export LD_LIBRARY_PATH=/home/hsy/桌面/dang/build/run
while true; do
        server=ps aux | grep CenterServer_d | grep -v grep
        if [ ! "$server" ]; then
            /home/hsy/桌面/dang/build/run 
	    echo "restart"
            sleep 10
        fi
        sleep 5
done
