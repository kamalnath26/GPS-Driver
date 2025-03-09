#!/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import Header
from gps_driver.msg import Customgps
import sys
import utm
import time 
from datetime import datetime, timezone

class GPS_DRIVER():
    
    def __init__(self, port="/dev/ttyUSB0", baudrate="4800", time_out=5):
        self.serial_port = port
        self.serial_port_baudrate = baudrate
        self.serial_port_time_out = time_out
        self.sequence_number = 0
        self.node = rospy.init_node('gps_driver')
        self.publisher = rospy.Publisher('/gps', Customgps, queue_size=10)
        self.rate = rospy.Rate(1)
        
    def connect(self):
        '''Try to connect to the serial port of GPS.'''
        try:
            rospy.loginfo("Connecting to GPS")
            self.gps_serial = serial.Serial(self.serial_port, self.serial_port_baudrate, timeout=self.serial_port_time_out)
            return True
        except serial.SerialException as error:
            rospy.loginfo("Shutting down...")
            rospy.signal_shutdown("Keyboard Interrupt")
            rospy.logerr_once(f"Unexpected error happened: {error}")
            sys.exit()

    def isGPGGAinString(self, inputString):
        '''Check whether the input string is GPGGA or not
        Args: Input String -> Data must be a string 
        Returns: True if the string is GPGGA, False otherwise'''
        if len(inputString) == 0 or not inputString[0].startswith("$GPGGA"):
            rospy.logdebug(f"NO GPGGA String found in {inputString}")
            return True
        else:
            rospy.loginfo(f"Verified GPGGA String found in {inputString}")
            return False
    
    def update_raw_data(self):
        '''Update raw GPS data from the split string.'''
        if len(self.raw_gps_data_split) < 15:
            rospy.logdebug("Not enough data points to update.")
            return [None] * 7  # Return a list with None values

        try:
            self.utc = self.raw_gps_data_split[1] if self.raw_gps_data_split[1] else 0.0
            self.latitude = self.raw_gps_data_split[2] if self.raw_gps_data_split[2] else 0.0
            self.latitudeDir = self.raw_gps_data_split[3] if self.raw_gps_data_split[3] else None
            self.longitude = self.raw_gps_data_split[4] if self.raw_gps_data_split[4] else 0.0
            self.longitudeDir = self.raw_gps_data_split[5] if self.raw_gps_data_split[5] else None
            self.altitude = float(self.raw_gps_data_split[9]) if self.raw_gps_data_split[9] else 0.0
            self.hdop = float(self.raw_gps_data_split[8]) if self.raw_gps_data_split[8] else 0.0
        except ValueError as e:
            rospy.logerr(f"ValueError: {e}")
            return [0.0] * 7  # Return a list with default values

        return [self.utc, self.latitude, self.latitudeDir, self.longitude, self.longitudeDir, self.altitude, self.hdop]
       
    def degMinstoDegDec(self, LatOrLong, is_latitude=True):
        '''Converts latitude or longitude from DDmm.mm to DD.dddd format.'''
        if is_latitude:
            deg = int(LatOrLong[:2])  
            mins = float(LatOrLong[2:])
        else:
            deg = int(LatOrLong[:3])  
            mins = float(LatOrLong[3:])  
        return deg + (mins / 60.0)
        
    def LatLongSignConvetion(self, LatOrLong, LatOrLongDir):
        '''Converts latitude or longitude to signed value based on direction.'''
        if LatOrLongDir in ['S', 'W']:  
            return -1 * LatOrLong  
        else:
            return LatOrLong

    def convertToUTM(self, LatitudeSigned, LongitudeSigned):
        '''Converts latitude and longitude to UTM coordinates.'''
        UTMVals = utm.from_latlon(LatitudeSigned, LongitudeSigned)
        UTMEasting = UTMVals[0]
        UTMNorthing = UTMVals[1]
        UTMZone = UTMVals[2]
        UTMLetter = UTMVals[3]
        return [UTMEasting, UTMNorthing, UTMZone, UTMLetter]

    def UTCtoUTCEpoch(self, UTC):
        '''Converts UTC time to epoch time.'''
        try:
            hours = int(UTC[:2])
            minutes = int(UTC[2:4])
            seconds = float(UTC[4:])
            
            now = datetime.utcnow()
            utc_time = now.replace(hour=hours, minute=minutes, second=int(seconds), microsecond=int((seconds % 1) * 1e6))
            
            CurrentTimeSec = int(utc_time.timestamp())
            CurrentTimeNsec = int((utc_time.timestamp() - CurrentTimeSec) * 1e9)
            return [CurrentTimeSec, CurrentTimeNsec]
        except Exception as e:
            rospy.logerr(f"Error converting UTC to epoch: {e}")
            return [0, 0]

    def update_ros_msg_and_publish(self):
        '''Update and publish the ROS message.'''
        self.msg = Customgps()
        self.msg.header = Header()
        self.msg.header.seq = self.sequence_number
        self.sequence_number += 1 
        self.msg.header.frame_id = 'GPS1_Frame'
        self.msg.header.stamp.secs = self.CurrentTimeSec
        self.msg.header.stamp.nsecs = self.CurrentTimeNsec
        self.msg.latitude = self.latitude_signed if self.latitude_signed is not None else 0.0
        self.msg.longitude = self.longitude_signed if self.longitude_signed is not None else 0.0
        self.msg.altitude = self.altitude
        self.msg.utm_easting = self.UTM_eastings
        self.msg.utm_northing = self.UTM_northings
        self.msg.zone = self.UTM_zone
        self.msg.letter = self.UTM_letter
        self.msg.hdop = self.hdop
        self.msg.gpgga_read = self.raw_gps_data
            
        rospy.loginfo(self.msg)
        self.publisher.publish(self.msg)
        self.rate.sleep()
    
    def get_gps_data(self):
        '''Get data from serial port and return it as a string.'''
        try:
            self.raw_gps_data = self.gps_serial.readline().decode().strip()
            self.raw_gps_data_split = self.raw_gps_data.split(",")
            rospy.loginfo(f"Received raw GPS data: {self.raw_gps_data}")
        except Exception as error:
            rospy.logerr(f"Data parsing error: {error}")
    
    def start_driver(self):
        '''Main loop for reading GPS data and publishing it.'''
        while not rospy.is_shutdown():
            self.get_gps_data()
            if self.isGPGGAinString(self.raw_gps_data_split):
                continue
            if len(self.raw_gps_data_split) < 15:
                rospy.logdebug("Missing a few data points, skipping this iteration")
                continue
            self.updated_values = self.update_raw_data()
            if None in self.updated_values or 0.0 in self.updated_values:
                rospy.logdebug("Missing a few data points, skipping this iteration")
                continue
            
            self.latitude_degrees = self.degMinstoDegDec(self.latitude, is_latitude=True)
            self.longitude_degrees = self.degMinstoDegDec(self.longitude, is_latitude=False)
            self.latitude_signed = self.LatLongSignConvetion(self.latitude_degrees, self.latitudeDir)
            self.longitude_signed = self.LatLongSignConvetion(self.longitude_degrees, self.longitudeDir)

            rospy.loginfo(f"Converted Latitude: {self.latitude_signed}, Converted Longitude: {self.longitude_signed}")
            self.UTM_eastings, self.UTM_northings, self.UTM_zone, self.UTM_letter = self.convertToUTM(self.latitude_signed, self.longitude_signed)

            rospy.loginfo(f"UTM conversion: UTM_easting: {self.UTM_eastings}, UTM_northing: {self.UTM_northings}, UTM_zone: {self.UTM_zone}, UTM_letter: {self.UTM_letter}")
            self.CurrentTimeSec, self.CurrentTimeNsec = self.UTCtoUTCEpoch(self.utc)
            self.update_ros_msg_and_publish()

if __name__ == "__main__":
    try:
        if len(sys.argv) == 2:
            port_number = sys.argv[1]
        else:
            port_number = rospy.get_param('~port', '/dev/ttyUSB0')
        gps_baudrate = rospy.get_param('~baudrate', '4800')
        connection_timeout = rospy.get_param('~connection_timeout', 5)
        rospy.loginfo(f"Starting the GPS Driver with port {port_number}, baudrate of {gps_baudrate}, and connection timeout of {connection_timeout}")
        gps = GPS_DRIVER(port=port_number, baudrate=gps_baudrate, time_out=connection_timeout)
        gps.connect()
        gps.start_driver()
    except Exception as error:
        rospy.loginfo(f"Error: {error}")