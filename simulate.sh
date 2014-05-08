#!/usr/bin/env bash

# COMPILE CODE AND START THE SIMULATION WITH TERMINAL FOR BOTH ROBOTS #

if [ "$1" == "" ] 
then
echo "Please select the code you want to compile: main/sec"
exit
fi

if [ "$1" == "sec" ] 
then
cd ~/Eurobotics_software_2014/secondary_robot/
else
cd ~/Eurobotics_software_2014/maindspic/
fi
make H=1
python ~/Eurobotics_software_2014/maindspic/display.py &

gnome-terminal --title="SECONDARY ROBOT" --tab -e "bash -c ~/Eurobotics_software_2014/maindspic/main H=1" gnome-terminal --title="MAIN ROBOT" --tab -e "bash -c ~/Eurobotics_software_2014/secondary_robot/main H=1" 
