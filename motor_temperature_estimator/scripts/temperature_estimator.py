#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from urdf_parser_py.urdf import *
from sensor_msgs.msg import JointState
from motor_temperature_estimator_msgs.msg import MotorTemperature
from motor_temperature_estimator_msgs.cfg import MotorThermalParamConfig
from motor_temperature_estimator import estimator


class MotorTemperatureEstimator():
    def __init__(self):
        urdf = URDF().parse(rospy.get_param("robot_description"))
        self.estimators = {}
        for joint_name in urdf.joint_map.keys():
            self.estimators[joint_name] = {"server" : Server(type=MotorThermalParamConfig,
                                                             callback=lambda config, level: config,
                                                             namespace="~"+joint_name),
                                           "Tcoil" : 25.0,
                                           "Thousing" : 25.0,
                                           "last_update" : rospy.Time.now()}
        rospy.Subscriber("joint_states", JointState, self.jointStateCallback)
        self.pub = rospy.Publisher('~output', MotorTemperature, queue_size=10)
        self.seq = 0
        rospy.Timer(rospy.Duration(1.0/50), self.timerCallback)

    def jointStateCallback(self,msg):
        if len(msg.name) != len(msg.effort):
            rospy.logwarn("len(msg.name) != len(msg.effort)")
            return
        for joint_name, joint_effort in zip(msg.name, msg.effort):
            if not joint_name in self.estimators:
                rospy.logwarn(joint_name + " not found")
            est = self.estimators[joint_name]
            est["Tcoil"], est["Thousing"] = estimator.estimateTemperature(Tcoil = est["Tcoil"],
                                                                          Thousing = est["Thousing"],
                                                                          Tair = 25.0,
                                                                          thermal_param = est["server"].config,
                                                                          tau = joint_effort,
                                                                          dt = (msg.header.stamp - est["last_update"]).to_sec())
            est["last_update"] = msg.header.stamp

    def timerCallback(self,event):
        msg = MotorTemperature()
        msg.header.seq = self.seq
        msg.header.stamp = rospy.Time.now()
        for joint_name in self.estimators.keys():
            est = self.estimators[joint_name]
            msg.name.append(joint_name)
            msg.coil.append(est["Tcoil"])
            msg.housing.append(est["Thousing"])
        self.pub.publish(msg)
        self.seq += 1

if __name__ == "__main__":
    rospy.init_node("temperature_estimator")
    worker = MotorTemperatureEstimator()
    rospy.spin()
