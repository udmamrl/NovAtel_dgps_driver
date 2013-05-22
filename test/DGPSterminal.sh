# -e: with local echo , -D: with Hex print for none ASCII char. 
#miniterm.py /dev/ttyUSB0 -b 115200 -e -D
# use this script will auto connect to programed DGPS port
# /dev/sensors/ftdi_THORDGPS or /dev/sensors/ftdi_CAP_DGPS

# find the port
port=$(ls /dev/sensors | grep DGPS)
echo Found GPS port at /dev/sensors/$port

#copy the port 
sudo cp -prv /dev/sensors/$port /dev/sensors/DGPS

# delete it so ros can't find it
sudo rm /dev/sensors/$port

# kill the gps node , so no one is using serial port
rosnode kill /gps/NovAtel_dgps

# some delay
sleep 1

# connect to GPS Serial port
miniterm.py /dev/sensors/$(ls /dev/sensors | grep DGPS) -b 115200 -e -D

# after finish , put the port back
sudo cp -prv /dev/sensors/DGPS /dev/sensors/$port

# remove the link
sudo rm /dev/sensors/DGPS
 

