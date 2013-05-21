#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# This file is part of Fork of ros-drivers/nmea_gps_driver
# Modify for NovAtel ProPak V3/ LB+ DGPS unit
# Copyright (c) 2013 UDM Advanced Mobile Robotics Lab Cheng-Lung Lee
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Steven Martin, Eric Perko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib
roslib.load_manifest('NovAtel_dgps_driver')
import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from sensor_msgs.msg import TimeReference
from geometry_msgs.msg import TwistStamped

import serial, string, math, time, calendar
#global checksum_error_counter
#global  Checksum_error_limits
#nmea_utc should be a string of form hhmmss
def convertNMEATimeToROS(nmea_utc):
    #Get current time in UTC for date information
    utc_struct = time.gmtime() #immutable, so cannot modify this one
    utc_list = list(utc_struct)
    hours = int(nmea_utc[0:2])
    minutes = int(nmea_utc[2:4])
    seconds = int(nmea_utc[4:6])
    utc_list[3] = hours
    utc_list[4] = minutes
    utc_list[5] = seconds
    unix_time = calendar.timegm(tuple(utc_list))
    return rospy.Time.from_sec(unix_time)

#Add the tf_prefix to the given frame id
def addTFPrefix(frame_id):
    prefix = ""
    prefix_param = rospy.search_param("tf_prefix")
    if prefix_param:
        prefix = rospy.get_param(prefix_param)
        if prefix[0] != "/":
            prefix = "/%s" % prefix

    return "%s/%s" % (prefix, frame_id)

#Check the NMEA sentence checksum. Return True if passes and False if failed
def check_checksum(nmea_sentence):
    global checksum_error_counter, Checksum_error_limits
    try:
        split_sentence = nmea_sentence.split('*')
        transmitted_checksum = split_sentence[1].strip()
        
        #Remove the $ at the front
        data_to_checksum = split_sentence[0][1:]
        checksum = 0
        for c in data_to_checksum:
            checksum ^= ord(c)
        result=("%02X" % checksum)  == transmitted_checksum.upper()
        if result:
            # if we got right checksum, reset error counter
            checksum_error_counter=0
        else:
            # keep checksum error count
            checksum_error_counter+=1

        return result
    except:
        # Usually binary data will cause error.
        checksum_error_counter+=1
 
        return False

def Novatel_unlogall(com):
    global GPS
    Config_str1=('\r\nunlogall\r\nlog loglist\r\n' )
    # read all data out
    # data=GPS.read(GPS.inWaiting())
    #write loglist
    GPS.write(Config_str1) # unlog
    time.sleep(0.1)
    print '\r\nunlog start\r\n'
    # Read every line till no data in buffer
    while (GPS.inWaiting() >0):
        data = GPS.readline()
        #print data
        #example log list
        #<OK
        #[COM1]<LOGLIST COM1 0 89.0 UNKNOWN 0 790.529 004c0020 c00c 5683
        #<     9 
        #<          COM1 RXSTATUSEVENTA ONNEW 0.000000 0.000000 HOLD 
        #<          COM2 RXSTATUSEVENTA ONNEW 0.000000 0.000000 HOLD 
        #<          COM3 RXSTATUSEVENTA ONNEW 0.000000 0.000000 HOLD 
        #<          COM1_30 PSRDOPB ONCHANGED 0.000000 0.000000 HOLD 
        #<          COM1_30 TRACKSTATB ONTIME 1.000000 0.000000 HOLD
        #<          COM1_30 BESTPOSB ONTIME 1.000000 0.000000 HOLD
        #<          COM1_30 SATVISB ONTIME 1.000000 0.000000 HOLD
        #<          COM1 LOGLIST ONCE 0.000000 0.000000 NOHOLD
        #[COM1] 
        #data='<          COM1 RXSTATUSEVENTA ONNEW 0.000000 0.000000 HOLD'     
        if ( ( com in data) & (( 'ONNEW' in data) | ( 'ONCHANGED' in data) | ('ONTIME' in data)) ): 
            fields = data.split(' ');
            # fields[10] , fields[11] , this will unlog com1_30 data too.
            unlog_str=('unlog '+fields[10]+' '+fields[11]+'\r\n')
            GPS.write( unlog_str)
            rospy.loginfo( 'Send unlog command to DGPS:'+unlog_str) 
   
    rospy.loginfo('Unlog alldata from DGPS %s' % com )  
        
def _shutdown():
    global GPS
    print "DGPS shutdown time!"
    GPS.close() #Close DGPS serial port

if __name__ == "__main__":
    global Checksum_error_limits
    global checksum_error_counter
    global GPS
    #init publisher
    rospy.init_node('NovAtel_dgps_driver')
    gpspub = rospy.Publisher('fix', NavSatFix)
    gpstimePub = rospy.Publisher('time_reference', TimeReference)
    #Init GPS port
    GPSport = rospy.get_param('~port','/dev/ttyUSB0')
    GPSrate = rospy.get_param('~baud',115200)
    frame_id = rospy.get_param('~frame_id','gps')
    if frame_id[0] != "/":
        frame_id = addTFPrefix(frame_id)

    time_ref_source = rospy.get_param('~time_ref_source', frame_id)
    
    # for NovAtel DGPS plase set this False, use gpggalong data
    useRMC = rospy.get_param('~useRMC', False)
    
    # these are for NovAtel dgps setting
    # The update rate for the DGPS, NovAtel ProPak V3 20Hz, LB+ 10 Hz.
    gps_update_rate = rospy.get_param('~gps_update_rate', 10.)    
    # this is the serial port number on NovAtel DGPS unit not your PC's.
    NovAtel_output_port=rospy.get_param('~NovAtel_output_port', 'COM1')
    Checksum_error_limits=rospy.get_param('~Checksum_error_limits', 10.)
    NovAtel_SAVECONFIG=rospy.get_param('~NovAtel_SAVECONFIG',False)
    #useRMC == True -> generate info from RMC+GSA
    #useRMC == False -> generate info from GGA
    navData = NavSatFix()
    if useRMC:
        gpsVelPub = rospy.Publisher('vel',TwistStamped)
        gpsVel = TwistStamped()
        gpsVel.header.frame_id = frame_id
    gpstime = TimeReference()
    gpstime.source = time_ref_source
    navData.header.frame_id = frame_id
    GPSLock = False
    checksum_error_counter=0
    rospy.on_shutdown(_shutdown)
    try:
        GPS = serial.Serial(port=GPSport, baudrate=GPSrate, timeout=0.5)
        #Setup DGPS here
        #unlogall COM1
        #unlogall COM1_30
        #RTKSOURCE OMNISTAR
        #PSRDIFFSOURCE OMNISTAR
        #log gprmc ontime .05
        #log gpgsa ontime .05
        #log gpggalong ontime .05
        #SAVECONFIG


        Config_str1=('\r\nunlogall\r\n')
        Config_str2=('log gpggalong ontime %1.2f\r\n' % (1./gps_update_rate)  )
        Config_str3=('log gprmc ontime %1.2f\r\n' % (1./gps_update_rate)  )
        Config_str4=('log gpgsa ontime %1.2f\r\n' % (1./gps_update_rate)  )
        Config_str5='RTKSOURCE OMNISTAR\r\n'
        Config_str6='PSRDIFFSOURCE OMNISTAR\r\n'
        Config_str7='SAVECONFIG\r\n'

        #run our unlog will unlog unwant binary data
        Novatel_unlogall(NovAtel_output_port)
        ### for loop back testing only
        #GPS.write('OK')
        ### end of loop back debug
        GPS.write(Config_str1) # unlog streaming data, just try to test the communication
        GPS.flush() # flush data out
        time.sleep(0.1)
        if (GPS.inWaiting() >0):
            #read out all datas, the response shuldbe OK
            data=GPS.read(GPS.inWaiting())
            rospy.loginfo("Send %s to DGPS. Got: %s" % (Config_str1 ,data)) 
            if ('OK' in data):
                rospy.loginfo("Got OK when unlog DGPS data")
            else: 
                rospy.logerr('[DGPS] Unable to unlog DGPS data. Shutdown!')
                rospy.signal_shutdown('Unable to unlog DGPS data')
 
        else:
            #sned error no data in buffer error
            rospy.logerr('[DGPS]Received No data from DGPS. Shutdown!')
            rospy.signal_shutdown('Received No data from DGPS')

        # set up output formate, for NovAtel dpgs please use gpggalong
        if useRMC:
            GPS.write(Config_str3)
            data = GPS.readline();data = GPS.readline(); #print data
            rospy.loginfo('[DGPS] Send: '+Config_str3+'Got:'+data)
            GPS.write(Config_str4)
            data = GPS.readline();data = GPS.readline(); #print data
            rospy.loginfo('[DGPS] Send: '+Config_str4+'Got:'+data)
        else:
            GPS.write(Config_str2) # log gpggalong
            data = GPS.readline();data = GPS.readline(); #print data
            rospy.loginfo('[DGPS] Send: '+Config_str2+'Got:'+data)
            
        # setup DGPS
        GPS.write(Config_str5) #RTKSOURCE OMNISTAR
        data = GPS.readline();data = GPS.readline(); #print data
        rospy.loginfo('[DGPS] Send: '+Config_str5+'Got:'+data)
        GPS.write(Config_str6) #PSRDIFFSOURCE OMNISTAR
        data = GPS.readline();data = GPS.readline(); #print data
        rospy.loginfo('[DGPS] Send: '+Config_str6+'Got:'+data)
        if (NovAtel_SAVECONFIG):
            GPS.write(Config_str7) #SAVECONFIG 
            data = GPS.readline();data = GPS.readline(); #print data
            rospy.loginfo('[DGPS] Send: '+Config_str7+'Got:'+data)
        data = GPS.readline() # drop first output
        while not rospy.is_shutdown():
        
        
        
            ### for loop back debug only
            #StrGPGGA='$GPGGA,231127.10,4224.7880856,N,08308.2865031,W,9,04,3.1,210.111,M,-34.04,M,03,0138*64\r\n'
            #StrGPGGA='FakeBinaryData$GPGGA,231127.10,4224.7880856,N,08308.2865031,W,9,04,3.1,210.111,M,-34.04,M,03,0138*64\r\n'
            #StrGPGGA0='$GPGGA,,,,,,0,,,,,,,,*66\r\n'
            #StrGPGGA1='$GPGGA,,,,,,0,,,,,,,,*00\r\n'
            #GPS.write(StrGPGGA)
            #time.sleep(0.3)
            ### end of loop back debug
            
            
            #read GPS line
            data = GPS.readline()
            
            # Try to filter out binary data here
            # If we do proper setup , no binary data will show up , just in case ... I like to be prepared.
            if (len(data)>0) : # if is not empty string
                if (data[0]!='$'): # if $ is not at the start of the array
                    #rospy.logerr("[DGPS] Looks like we have binary data in the output. Please do log loglist and fix it!")
                    rospy.logwarn("[DGPS] Received a sentence but not start with $. Sentence was: %s .Fixing it!" % data)
                    fields = data.partition('$') # split the string with $
                    data=fields[1]+fields[2]     # put nema string back
                    # fields[0] should be binary data or [COM1]


            if not check_checksum(data):
                #print checksum_error_counter
                #print Checksum_error_limits
                #print (checksum_error_counter > Checksum_error_limits )
                rospy.logerr("[DGPS] Received a sentence with an invalid checksum. Sentence was: %s" % data)
                if (checksum_error_counter > Checksum_error_limits ):
                    #shutdown DGPS node
                    rospy.logfatal('[DGPS] Too much back to back checksumn error in DGPS data. Shutdown!')
                    rospy.signal_shutdown('Too much back to back checksum error in DGPS data')
                    
                continue

            timeNow = rospy.get_rostime()
            fields = data.split(',')
            for i in fields:
                i = i.strip(',')
            try:
                if useRMC:
                    #Check for satellite lock
                    if 'GSA' in fields[0]:
                        lockState = int(fields[2])
                        #print 'lockState=',lockState
                        if lockState == 3:
                            GPSLock = True
                        else:
                            GPSLock = False
                    #if not satellite lock message parse it separately
                    else:
                        if GPSLock == True:
                            if 'RMC' in fields[0]:
                                #print fields
                                gpsVel.header.stamp = timeNow
                                gpsVel.twist.linear.x = float(fields[7])*0.514444444444*math.sin(float(fields[8]))
                                gpsVel.twist.linear.y = float(fields[7])*0.514444444444*math.cos(float(fields[8]))
                                gpsVelPub.publish(gpsVel)

                                navData.status.status = NavSatStatus.STATUS_FIX
                                navData.header.stamp = gpsVel.header.stamp
                                navData.status.service = NavSatStatus.SERVICE_GPS

                                gpstime.header.stamp = gpsVel.header.stamp
                                gpstime.time_ref = convertNMEATimeToROS(fields[1])

                                longitude = float(fields[5][0:3]) + float(fields[5][3:])/60
                                if fields[6] == 'W':
                                    longitude = -longitude

                                latitude = float(fields[3][0:2]) + float(fields[3][2:])/60
                                if fields[4] == 'S':
                                    latitude = -latitude

                                #publish data
                                navData.latitude = latitude
                                navData.longitude = longitude
                                navData.altitude = float('NaN')
                                navData.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                                gpspub.publish(navData)
                                gpstimePub.publish(gpstime)
                        else:
                            pass
                            #print data
                else:
                    #Use GGA
                    #No /vel output from just GGA
                    if 'GGA' in fields[0]:
                        gps_quality = int(fields[6])
                        ##NovAtel DGPS Quality indicator
                        #    0 = fix not available or invalid
                        #    1 = GPS fix
                        #    2 = C/A differential GPS, OmniSTAR HP,
                        #        OmniSTAR XP, OmniSTAR VBS
                        #    4 = RTK fixed ambiguity solution (RT2)
                        #    5 = RTK floating ambiguity solution (RT20),
                        #        OmniSTAR HP or OmniSTAR XP
                        #    6 = Dead reckoning mode
                        #    7 = Manual input mode (fixed position)
                        #    8 = Simulator mode
                        #    9 = WAAS
                        #
                        #
                        if gps_quality == 0:        # no fix (-1)
                            navData.status.status = NavSatStatus.STATUS_NO_FIX
                        elif gps_quality in (1,9):  # fix or WAAS (0)
                            navData.status.status = NavSatStatus.STATUS_FIX
                        elif gps_quality == 2:      # DGPS  (1)
                            navData.status.status = NavSatStatus.STATUS_SBAS_FIX
                        elif gps_quality in (4,5):  # RTK (2)
                            #Maybe 2 should also sometimes be GBAS... but pretty
                            #sure RTK has to have a base station
                            navData.status.status = NavSatStatus.STATUS_GBAS_FIX
                        else:
                            navData.status.status = NavSatStatus.STATUS_NO_FIX
                        navData.status.service = NavSatStatus.SERVICE_GPS

                        navData.header.stamp = timeNow

                        latitude = float(fields[2][0:2]) + float(fields[2][2:])/60
                        if fields[3] == 'S':
                            latitude = -latitude
                        navData.latitude = latitude

                        longitude = float(fields[4][0:3]) + float(fields[4][3:])/60
                        if fields[5] == 'W':
                            longitude = -longitude
                        navData.longitude = longitude

                        hdop = float(fields[8])
                        navData.position_covariance[0] = hdop**2
                        navData.position_covariance[4] = hdop**2
                        navData.position_covariance[8] = (2*hdop)**2 #FIX ME
                        navData.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

                        #Altitude is above ellipsoid, so adjust for mean-sea-level
                        altitude = float(fields[9]) + float(fields[11])
                        navData.altitude = altitude


                        gpstime.header.stamp = timeNow
                        gpstime.time_ref = convertNMEATimeToROS(fields[1])

                        gpspub.publish(navData)
                        gpstimePub.publish(gpstime)
            except ValueError as e:
                rospy.logwarn("Value error, likely due to missing fields in the NMEA messages. Error was: %s" % e)

    except rospy.ROSInterruptException:
        GPS.close() #Close GPS serial port
