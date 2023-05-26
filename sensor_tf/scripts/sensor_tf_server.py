#!/usr/bin/env python


# import the SensorTransform service
from __future__ import print_function
import rospy
import tf2_ros
import geometry_msgs.msg
from gs_srv.srv import SensorTransform, SensorTransformResponse

NAME = "sensor_tf_server"


class SensorTransformServer(object):
    def __init__(self):
        self.x = 3.0
        self.y = 3.0
        self.z = 0.5
        self.parent_id = "map"
        self.anemometer_id = "anemometer_frame"
        self.PID_id = "pid_frame"

        self._srv = rospy.Service("sensor_transform", SensorTransform, self.service_handler, 10)
        print("Ready to transform sensor data.")

    def service_handler(self, req):
        self.x = req.x
        self.y = req.y
        self.z = req.z

        resp = SensorTransformResponse()
        return resp

    def transform_publisher(self):
        pid_br = tf2_ros.TransformBroadcaster()
        mox0_br = tf2_ros.TransformBroadcaster()
        mox1_br = tf2_ros.TransformBroadcaster()
        mox2_br = tf2_ros.TransformBroadcaster()
        anemometer_br = tf2_ros.TransformBroadcaster()

        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.parent_id
        t.child_frame_id = self.PID_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z
        t.transform.rotation.x = 0
        t.transform.rotation.y = 0
        t.transform.rotation.z = 0
        t.transform.rotation.w = 1
        t.header.stamp = rospy.Time.now()

        pid_br.sendTransform(t)

        t.child_frame_id = self.anemometer_id
        anemometer_br.sendTransform(t)

        t.child_frame_id = "mox0_frame"
        mox0_br.sendTransform(t)

        t.child_frame_id = "mox1_frame"
        mox1_br.sendTransform(t)

        t.child_frame_id = "mox2_frame"
        mox2_br.sendTransform(t)


if __name__ == "__main__":
    rospy.init_node(NAME)
    server = SensorTransformServer()

    loop_rate = rospy.Rate(2)
    while rospy.is_shutdown() is False:
        server.transform_publisher()
        loop_rate.sleep()
