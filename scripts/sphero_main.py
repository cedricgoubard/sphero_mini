#! /usr/bin/python3
import json

import rospy
from box import Box

from sphero_mini.simple_moves import forward
from sphero_mini.core import SpheroMini

def connect():
    conf_file_path = rospy.get_param("/conf_file_path")
    with open(conf_file_path, 'r') as f:
        cfg = Box(json.load(f))

    # Connect:
    sphero = SpheroMini(cfg.MAC_ADDRESS, verbosity = 1)
    # battery voltage
    sphero.getBatteryVoltage()
    print(f"Bettery voltage: {sphero.v_batt}v")

    # firmware version number
    sphero.returnMainApplicationVersion()
    print(f"Firmware version: {'.'.join(str(x) for x in sphero.firmware_version)}")
    return sphero

def disconnect(sphero):
    sphero.sleep()
    sphero.disconnect()

def main(sphero):
    rospy.init_node('sphero', anonymous=True)   
    rate = rospy.Rate(10)  # 10 Hz

    sphero.configureSensorMask(
        IMU_yaw=True,
        IMU_pitch=True,
        IMU_roll=True,
        IMU_acc_y=True,
        IMU_acc_z=True,
        IMU_acc_x=True,
        IMU_gyro_y=True,
        IMU_gyro_z=True 
        )
    sphero.configureSensorStream()

    while not rospy.is_shutdown():
        print(sphero.__dict__)
        print("IMU_pitch", sphero.IMU_pitch)
        print("IMU_roll", sphero.IMU_roll)
        print("IMU_yaw", sphero.IMU_yaw)
        print("IMU_acc_y", sphero.IMU_acc_y)
        print("IMU_acc_z", sphero.IMU_acc_z)
        print("IMU_acc_x", sphero.IMU_acc_x)
        print("IMU_gyro_y", sphero.IMU_gyro_y)
        print("IMU_gyro_x", sphero.IMU_gyro_x)
        rate.sleep()


if __name__ == "__main__":
    sphero = connect()
    try:
        main(sphero)
    except Exception as e: # rospy.ROSInterruptException
        disconnect(sphero)
        raise e
    