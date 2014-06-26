# -e: with local echo , -D: with Hex print for none ASCII char. 
#miniterm.py /dev/ttyUSB0 -b 115200 -e -D
# use this script will auto connect to programed DGPS port
# /dev/sensors/ftdi_THORDGPS or /dev/sensors/ftdi_CAP_DGPS

# find the port
#port=$(ls /dev/sensors | grep DGPS)
# find the port from ros parameter setting
port=$(rosparam get /gps/NovAtel_dgps/port)
echo Found GPS port at $port

#copy the port 
sudo cp -prv $port /dev/sensors/DGPS

# delete it so ros can't find it
sudo rm $port

# kill the gps node , so no one is using serial port
rosnode kill /gps/NovAtel_dgps

# some delay
sleep 1

# connect to GPS Serial port
miniterm.py /dev/sensors/DGPS -b 115200 -e -D

# after finish , put the port back
sudo cp -prv /dev/sensors/DGPS $port

# remove the link
sudo rm /dev/sensors/DGPS
 

