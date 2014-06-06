#!/usr/bin/env bash

# COMPILE CODE AND START THE SIMULATION WITH TERMINAL FOR BOTH ROBOTS #

if [ "$1" == "" ] 
then
echo "Please select the code you want to compile: main/sec"
exit
fi

if [ "$1" == "sec" ] 
then
cd /media/datos/Datos/Proyectos/ARC_Eurobotics/eurobotics/eurobot/eurobot2014/software/secondary_robot/
else
cd /media/datos/Datos/Proyectos/ARC_Eurobotics/eurobotics/eurobot/eurobot2014/software/maindspic/
fi
make H=1
python /media/datos/Datos/Proyectos/ARC_Eurobotics/eurobotics/eurobot/eurobot2014/software/maindspic/display.py &

gnome-terminal --title="SECONDARY ROBOT" --tab -e "bash -c /media/datos/Datos/Proyectos/ARC_Eurobotics/eurobotics/eurobot/eurobot2014/software/maindspic/main H=1" gnome-terminal --title="MAIN ROBOT" --tab -e "bash -c /media/datos/Datos/Proyectos/ARC_Eurobotics/eurobotics/eurobot/eurobot2014/software/secondary_robot/main H=1" 
