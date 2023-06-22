# Copyright 2020 Evan Flynn
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
# from sensor_msgs import point_cloud2 
from sensor_msgs_py import point_cloud2

class PointCloudSubsriber(Node):
    def __init__(self):
        super().__init__('pc_publisher')

        self.pcd_subscriber = self.create_subscription(
            PointCloud2,    # Msg type
            '/intel_realsense_r200_depth/points',                      # topic
            self.listener_callback,      # Function to call
            10                           # QoS
        )

    def listener_callback(self, msg):
        # print("==============================================")
        # print("data: ", msg.data)
        print("=====================[Start]=========================")
        points = list(point_cloud2.read_points(msg, field_names=['x','y','z','intensity']))
        for point in points:
            print(point[0], point[1], point[2])
        print("=====================[End]=========================")


def main(args=None):
    rclpy.init(args=args)
    pc_subsriber = PointCloudSubsriber()
    rclpy.spin(pc_subsriber)
    pc_subsriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
