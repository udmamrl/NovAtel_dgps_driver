# -e: with local echo , -D: with Hex print for none ASCII char. 
#miniterm.py /dev/ttyUSB0 -b 115200 -e -D
# use this script will auto connect to programed DGPS port
# /dev/sensors/ftdi_THORDGPS or /dev/sensors/ftdi_CAP_DGPS
miniterm.py /dev/sensors/$(ls /dev/sensors | grep DGPS) -b 115200 -e -D

 

