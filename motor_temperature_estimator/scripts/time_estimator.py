#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from urdf_parser_py.urdf import *
from sensor_msgs.msg import JointState
from motor_temperature_estimator_msgs.msg import JointFloat64State
from motor_temperature_estimator_msgs.cfg import MotorThermalParamConfig
from motor_temperature_estimator import estimator

class TimeEstimator():
    def __init__(self):
        urdf = URDF().parse(rospy.get_param("robot_description"))
        self.estimators = {}
        for joint_name in urdf.joint_map.keys():
            if urdf.joint_map[joint_name].joint_type == 'fixed':
                continue
            self.estimators[joint_name] = {"server" : None,
                                           "tau" : 0.0,
                                           "Tcoil" : 25.0,
                                           "Thousing" : 25.0,
                                           "Tcoil_general" : 0.0,
                                           "Thousing_general" : 0.0}
            server = Server(type=MotorThermalParamConfig,
                            callback=lambda config, level, joint_name=joint_name : self.configCallback(config,joint_name),
                            namespace="~"+joint_name)
            new_config = {}
            for param_name in server.config:
                if rospy.has_param("~" + joint_name + "/" + param_name):
                    new_config[param_name] = rospy.get_param("~" + joint_name + "/" + param_name)
            server.update_configuration(new_config)
            self.estimators[joint_name]["server"] = server

        rospy.Subscriber("joint_states", JointState, self.jointStateCallback)
        rospy.Subscriber("~coil", JointFloat64State, self.coilTemperatureCallback)
        rospy.Subscriber("~housing", JointFloat64State, self.housingTemperatureCallback)
        self.timePub = rospy.Publisher('~time', JointFloat64State, queue_size=10)
        self.seq = 0
        rospy.Timer(rospy.Duration(10), self.timerCallback)

    def configCallback(self, config, joint_name):
        self.estimators[joint_name]["Tcoil_general"], self.estimators[joint_name]["Thousing_general"] = estimator.calcGeneralSolutionOfTemperature(config)
        return config

    def jointStateCallback(self,msg):
        if len(msg.name) != len(msg.effort):
            rospy.logwarn("len(msg.name) != len(msg.effort)")
            return
        for joint_name, joint_effort in zip(msg.name, msg.effort):
            if not joint_name in self.estimators:
                rospy.logwarn(joint_name + " not found")
            self.estimators[joint_name]["tau"] = joint_effort

    def coilTemperatureCallback(self,msg):
        if len(msg.name) != len(msg.data):
            rospy.logwarn("len(msg.name) != len(msg.data)")
            return
        for joint_name, coil_temperature in zip(msg.name, msg.data):
            if not joint_name in self.estimators:
                rospy.logwarn(joint_name + " not found")
            self.estimators[joint_name]["Tcoil"] = coil_temperature

    def housingTemperatureCallback(self,msg):
        if len(msg.name) != len(msg.data):
            rospy.logwarn("len(msg.name) != len(msg.data)")
            return
        for joint_name, housing_temperature in zip(msg.name, msg.data):
            if not joint_name in self.estimators:
                rospy.logwarn(joint_name + " not found")
            self.estimators[joint_name]["Thousing"] = housing_temperature

    def timerCallback(self,event):
        timeMsg = JointFloat64State()
        timeMsg.header.seq = self.seq
        timeMsg.header.stamp = rospy.Time.now()
        for joint_name in self.estimators.keys():
            est = self.estimators[joint_name]
            Tcoil, Thousing = estimator.calcSpecialSolutionOfTemperature(thermal_param = est["server"].config,
                                                                         Tcoil_general = est["Tcoil_general"],
                                                                         Thousing_general = est["Thousing_general"],
                                                                         Thousing_current = est["Thousing"],
                                                                         Tcoil_current = est["Tcoil"],
                                                                         Tair_current = 25.0)
            time = estimator.calcRemainingTimeFast(thermal_param = est["server"].config,
                                                   T_special = Tcoil,
                                                   tau_current = est["tau"])
            timeMsg.name.append(joint_name)
            timeMsg.data.append(time)

        self.timePub.publish(timeMsg)
        self.seq += 1

if __name__ == "__main__":
    rospy.init_node("time_estimator")
    worker = TimeEstimator()
    rospy.spin()
