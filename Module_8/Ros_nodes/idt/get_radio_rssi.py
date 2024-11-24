#!/usr/bin/env python3

# Introduction to Drone Technology IDT
# SDU UAS Center
# University of Southern Denmark
# 2024-11-13 Kjeld Jensen kjen@sdu.dk First version

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import NavSatFix
from threading import Thread, Lock
import csv

mutex = Lock()

class RSSIListener(Node):
    def __init__(self):
        super().__init__('get_radio_rssi')
        # Subscribe to the /diagnostics topic
        self.subscription = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.on_diagnostics,
            QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.get_logger().info("get_radio_rssi node Started...")

        self.sub_global_pos_raw = self.create_subscription(
                NavSatFix, # message type
                '/mavros/global_position/raw/fix', # topic to subscribe to
                self.on_global_pos_raw_msg, # callback function 
                QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            )
        self.gpsMessage=None
        self.RSSIMessage=None
        self.RRSSIMessage=None

    def on_diagnostics(self, msg):
        with mutex:
            for status in msg.status:
                if status.name == "mavros: 3DR Radio":
                    for value in status.values:
                        if value.key == "RSSI (dBm)":
                            rssi = value.value
                            #self.get_logger().info(f"Local RSSI (dBm): {rssi}")
                            self.RSSIMessage=(msg.header.stamp.sec,rssi)
                        elif value.key == "Remote RSSI (dBm)":
                            remote_rssi = value.value
                            #self.get_logger().info(f"Remote RSSI (dbm): {remote_rssi}")
                            self.RRSSIMessage=(msg.header.stamp.sec,remote_rssi)
                        if self.RSSIMessage != None and self.RRSSIMessage != None and self.gpsMessage!= None:
                            self.LogMessage()

    def on_global_pos_raw_msg(self, msg):
        with mutex:
            self.gpsMessage=(msg.header.stamp.sec,msg.latitude,msg.longitude,msg.altitude)
            if self.RRSSIMessage!= None and self.RSSIMessage != None:
                self.LogMessage()


    def LogMessage(self):
        gpstime=self.gpsMessage[0]
        latitude=self.gpsMessage[1]
        longitude=self.gpsMessage[2]
        altitude=self.gpsMessage[3]
        rssitime=self.RSSIMessage[0]
        rssi=self.RSSIMessage[1]
        rrssitime=self.RRSSIMessage[0]
        rrssi=self.RRSSIMessage[1]
        with open("gps_log.csv","a") as file:
            writer = csv.writer(file)

            writer.writerow([gpstime,latitude,longitude,altitude,rssi,rrssi])
            self.gpsMessage=None
            self.RSSIMessage=None
            self.RRSSIMessage=None

def main(args=None):
    rclpy.init(args=args)
    rssi_listener = RSSIListener()
    rclpy.spin(rssi_listener)

    # Destroy the node explicitly
    rssi_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

