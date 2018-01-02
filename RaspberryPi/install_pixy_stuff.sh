#!/bin/sh
cd /home/pi
apt-get install swig libusb-1.0-0-dev libboost-all-dev -y
pip install pynetworktables
if [! -d /home/pi/pixy];
then
	git clone https://github.com/charmedlabs/pixy.git
else
	printf "Already cloned pixy repository\n"
fi
if [! -d /home/pi/FRC2017];
then
	git clone https://github.com/team341/FRC2017.git;
fi
cd /home/pi/pixy/scripts
./build_libpixyusb_swig.sh
cd /home/pi/FRC2017/RaspberryPi
./copy.sh
./onstart.sh
reboot
