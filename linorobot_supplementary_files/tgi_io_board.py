#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt16

import serial
import numpy as np
import struct


def io_reader():
    serialComm = serial.Serial('/dev/ttyUSB1', 115200, timeout=0)
    incomingByte = bytes([])
    startFrame = b'\x02'
    stopFrame = b'\x03'
    no_received_bytes = 0
    feedback_bytes = bytes([])
    chk_sum = 0
    recv_chk_sum = 0
   
    pub_io = rospy.Publisher('/tgi_io_board/tgi_IO', UInt16, queue_size=10)
    pub_lidar1 = rospy.Publisher('/tgi_io_board/tgi_Lidar1', UInt16, queue_size=10)
    pub_lidar2 = rospy.Publisher('/tgi_io_board/tgi_Lidar2', UInt16, queue_size=10)
    pub_lidar3 = rospy.Publisher('/tgi_io_board/tgi_Lidar3', UInt16, queue_size=10)
    rospy.init_node('tgi_io_board', anonymous=True)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        if serialComm.is_open:
            if serialComm.in_waiting:
                incomingByte = serialComm.read(1)
                if incomingByte == startFrame:
                    no_received_bytes = 1
                    feedback_bytes = incomingByte
                else:
                    if no_received_bytes >= 1 and len(feedback_bytes) < 17:
                        feedback_bytes += incomingByte
                        no_received_bytes += 1
                        if no_received_bytes == 15:
                            chk_sum = np.uint8(sum(bytearray(feedback_bytes)))
                        elif no_received_bytes == 16:
                            recv_chk_sum = np.uint8(struct.unpack('B',incomingByte)[0])
                    if no_received_bytes == 17 and incomingByte == stopFrame:
                            #print(feedback_bytes, recv_chk_sum, chk_sum)
                            if( chk_sum == recv_chk_sum ):
                                io = int(feedback_bytes[1:3], 16)
                                lidar1 = int(feedback_bytes[3:7], 16)
                                lidar2 = int(feedback_bytes[7:11], 16)
                                lidar3 = int(feedback_bytes[11:15], 16)
                            
                                pub_io.publish(io)
                                pub_lidar1.publish(lidar1)
                                pub_lidar2.publish(lidar2)
                                pub_lidar3.publish(lidar3)
                            rate.sleep() 
            else:
                rate.sleep() 
    serialComm.close()
    
if __name__ == "__main__":
    try:
        io_reader()
    except rospy.ROSInterruptException:
        pass
