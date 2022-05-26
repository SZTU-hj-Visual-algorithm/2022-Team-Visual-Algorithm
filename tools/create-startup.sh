#!/bin/bash

echo "#!/bin/bash" > $2/startup-run

echo "echo dji | sudo -S cpufreq-set -g performance" >> $2/startup-run
echo "echo -e \"gnome-terminal -- bash -c \\\"$2/monitor.sh\\\"\">>~/.profile" >> $2/startup-run

echo "echo \"#!/bin/sh\" > ./monitor.sh" >> $2/startup-run
echo "echo \"$1/build/run\" >> ./monitor.sh" >> $2/startup-run
echo "echo \"export LD_LIBRARY_PATH=$1/build/run\" >> ./monitor.sh" >> $2/startup-run
echo "echo \"while true; do\" >> ./monitor.sh" >> $2/startup-run
echo "echo \"        server=ps aux | grep CenterServer_d | grep -v grep\">>./monitor.sh" >> $2/startup-run
echo "echo \"        if [ ! \\\"\\\$server\\\" ]; then\" >> ./monitor.sh" >> $2/startup-run
echo "echo \"            $1/build/run \" >> ./monitor.sh" >> $2/startup-run
echo "echo \"	    echo \\\"restart\\\"\" >> ./monitor.sh" >> $2/startup-run
echo "echo \"            sleep 10\" >> ./monitor.sh" >> $2/startup-run
echo "echo \"        fi\" >> ./monitor.sh" >> $2/startup-run
echo "echo \"        sleep 5\" >> ./monitor.sh" >> $2/startup-run
echo "echo \"done\" >> ./monitor.sh" >> $2/startup-run
echo "chmod +x ./monitor.sh" >> $2/startup-run

#echo "gnome-terminal -- bash -c \"echo sjturm | sudo -S $1/tools/monitor.sh \\\"$2/run --run-with-camera --save-video --save-mark --show-armor-box --wait-uart --save-labelled-boxes\\\"\"" >> $2/startup-run
chmod +x $2/startup-run
