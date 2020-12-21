#! /usr/bin/python3
import json

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
from box import Box
import numpy as np

from sphero_mini.simple_moves import forward
from sphero_mini.core import SpheroMini

def connect():
    conf_file_path = rospy.get_param("/conf_file_path")
    with open(conf_file_path, 'r') as f:
        cfg = Box(json.load(f))

    # Connect:
    sphero = SpheroMini(cfg.MAC_ADDRESS, verbosity = 4)
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


def create_quaternion(roll, pitch, yaw):
    q = Quaternion()
    cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
    cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
    cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)

    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy

    return q


def create_angular_veolocity_vector3(groll, gpitch, gyaw):
    v = Vector3()
    v.x = groll
    v.y = gpitch
    v.z = gyaw

    return v


def create_linear_acc_vector3(xacc, yacc, zacc):
    v = Vector3()
    v.x = xacc
    v.y = yacc
    v.z = zacc

    return v


def get_sensors_data(sphero):
    return {
        "roll": sphero.IMU_roll,
        "pitch": sphero.IMU_pitch,
        "yaw": sphero.IMU_yaw,
        "groll": sphero.IMU_gyro_x,
        "gpitch": sphero.IMU_gyro_y,
        "xacc": sphero.IMU_acc_x,
        "yacc": sphero.IMU_acc_y,
        "zacc": sphero.IMU_acc_z
    }
    

def publish_imu(pub, sensors_values):
    i = Imu()

    i.header.stamp = rospy.Time.now()
    i.orientation = create_quaternion(
        roll=sensors_values["roll"],
        pitch=sensors_values["pitch"],
        yaw=sensors_values["yaw"]
        )
    i.angular_velocity = create_angular_veolocity_vector3(
        groll=sensors_values["groll"],
        gpitch=sensors_values["gpitch"],
        gyaw=0  # We don't have the IMU_gyro_z
    )
    i.linear_acceleration = create_linear_acc_vector3(
        xacc=sensors_values["xacc"],
        yacc=sensors_values["yacc"],
        zacc=sensors_values["zacc"]
    )
    pub.publish(i)


def main(sphero):
    rospy.init_node('sphero', anonymous=True, log_level=rospy.DEBUG)   
    rate = rospy.Rate(10)  # 10 Hz

    pub = rospy.Publisher("/imu", Imu, queue_size=5)

    sphero.configureSensorMask(
        IMU_yaw=True,
        IMU_pitch=True,
        IMU_roll=True,
        IMU_acc_y=True,
        IMU_acc_z=True,
        IMU_acc_x=True,
        IMU_gyro_x=True,
        IMU_gyro_y=True,
        #IMU_gyro_z=True 
        )
    sphero.configureSensorStream()

    while not rospy.is_shutdown():
        sensors_values = get_sensors_data(sphero)
        #rospy.logdebug(sensors_values)
        publish_imu(pub, sensors_values)
        sphero.setLEDColor(red = 0, green = 255, blue = 0) # Turn LEDs green
        rate.sleep()


if __name__ == "__main__":
    sphero = connect()
    try:
        main(sphero)
    except Exception as e: # rospy.ROSInterruptException
        disconnect(sphero)
        raise e
    