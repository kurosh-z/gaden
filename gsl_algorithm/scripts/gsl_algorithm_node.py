from __future__ import print_function
import numpy as np
import scipy.interpolate as si
import matplotlib.pyplot as plt


import rospy
from gs_srv.srv import SensorTransform
from olfaction_msgs.msg import gas_sensor as gas_sensor_msg_t

NAME = "GLS_algorithm"

# pylint:disable=missing-function-docstring
# pylint:disable=missing-class-docstring
# pylint:disable=invalid-name

CV01 = np.array(
    [
        [9, 3],
        [8.5, 4],
        [7.5, 5],
        [7, 5],
        [7, 4],
        [7, 3],
        [7, 2],
        [8, 2],
        [9, 2],
        [9, 1],
        [8, 1],
        [7, 1],
        [6, 1],
        [5, 1],
        [3.6, 1],
        [3.6, 2],
        [3.6, 3],
        [3.6, 4],
        [3.6, 5],
        [2, 5],
        [2, 4],
        [2, 3],
        [2, 2],
        [2, 1],
        [0.9, 1],
        [0.9, 2],
        [0.9, 3],
        [0.9, 4],
        [0.9, 5],
    ]
)


class GLS_Algorithm(object):
    def __init__(self, num_of_points=100, height=0.4) -> None:
        self.height = height
        self.num_of_points = num_of_points
        self.curve_points = CV01
        self.set_position_ = rospy.ServiceProxy("/sensor_transform", SensorTransform)
        self.create_curve()
        self.current_position = None
        self.pid_measurements_list = []
        self.mox0_measurements_list = []
        self.mox1_measurements_list = []
        self.mox2_measurements_list = []

        rospy.Subscriber("PID/Sensor_reading", gas_sensor_msg_t, self.update_pid_measurements)
        rospy.Subscriber("Mox00/Sensor_reading", gas_sensor_msg_t, self.update_mox0_measurements)
        rospy.Subscriber("Mox01/Sensor_reading", gas_sensor_msg_t, self.update_mox1_measurements)
        rospy.Subscriber("Mox02/Sensor_reading", gas_sensor_msg_t, self.update_mox2_measurements)

    def bspline(self, cv, n=100, degree=3):
        """Calculate n samples on a bspline

        cv :      Array ov control vertices
        n  :      Number of samples to return
        degree:   Curve degree
        """
        cv = np.asarray(cv)
        count = cv.shape[0]

        # Prevent degree from exceeding count-1, otherwise splev will crash
        degree = np.clip(degree, 1, count - 1)

        # Calculate knot vector
        kv = np.array([0] * degree + list(range(count - degree + 1)) + [count - degree] * degree, dtype="int")

        # Calculate query range
        u = np.linspace(0, (count - degree), n)

        # Calculate result
        return np.array(si.splev(u, (kv, cv.T, degree))).T

    def create_curve(self):
        self.curve_2d_points = self.bspline(self.curve_points, n=self.num_of_points, degree=6)

    def get_to_the_node(self, node):
        try:
            self.set_position_(node[0], node[1], node[2])
            return True
        except rospy.ServiceException as exception:
            rospy.loginfo(f"Service call to set position failed: {exception}")
            return False

    def update_pid_measurements(self, data):
        self.pid_measurements_list.append({"pos": self.current_position, "pid": data})

    def update_mox0_measurements(self, data):
        self.mox0_measurements_list.append({"pos": self.current_position, "mox0": data})

    def update_mox1_measurements(self, data):
        self.mox1_measurements_list.append({"pos": self.current_position, "mox1": data})

    def update_mox2_measurements(self, data):
        self.mox2_measurements_list.append({"pos": self.current_position, "mox2": data})

    def collect_measurements(self):
        for idx in range(self.num_of_points):
            target_pos = [*self.curve_2d_points[idx, ...], self.height]
            rospy.loginfo(f"getting to the node x:{target_pos[0]} y:{target_pos[1]} z:{target_pos[2]}")
            resp = self.get_to_the_node(target_pos)
            if resp:
                self.current_position = target_pos
            rospy.sleep(0.5)

    def plot_raw_values(self, sensor_type):
        tt = []
        yy = []
        data_list = []
        if sensor_type == "pid":
            data_list = self.pid_measurements_list
        elif sensor_type == "mox0":
            data_list = self.mox0_measurements_list
        elif sensor_type == "mox1":
            data_list = self.mox1_measurements_list
        elif sensor_type == "mox2":
            data_list = self.mox2_measurements_list

        for data in data_list:
            tt.append(data[sensor_type].header.stamp.to_sec())
            yy.append(data[sensor_type].raw)
        plt.plot(tt, yy)
        plt.show()

    def save_to_file(self, file_name):
        for sensor_type in ["pid", "mox0", "mox1", "mox2"]:
            measurement_list = []
            if sensor_type == "pid":
                measurement_list = self.pid_measurements_list
            elif sensor_type == "mox0":
                measurement_list = self.mox0_measurements_list
            elif sensor_type == "mox1":
                measurement_list = self.mox1_measurements_list
            elif sensor_type == "mox2":
                measurement_list = self.mox2_measurements_list

            if len(measurement_list) != 0:
                try:
                    with open(
                        f"/home/kurosh/gaden_ws/src/gsl_algorithm/results/{file_name}_{sensor_type}.npy", "wb"
                    ) as f:
                        np.save(f, np.array(measurement_list))
                        rospy.loginfo(f"Successfully saved the data for {sensor_type}")
                except Exception as save_exception:
                    rospy.loginfo(f"Failed to save the data: {save_exception}")


if __name__ == "__main__":
    rospy.init_node(NAME)
    rospy.wait_for_service("sensor_transform")
    gls_alg = GLS_Algorithm()
    gls_alg.collect_measurements()
    gls_alg.save_to_file("central_w0_")
    gls_alg.plot_raw_values("mox0")
    print("finished")

    # loop_rate = rospy.Rate(2)
    # while rospy.is_shutdown() is False:
    #     server.transform_publisher()
    #     loop_rate.sleep()
