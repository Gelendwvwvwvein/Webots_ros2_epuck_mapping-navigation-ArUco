import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import time
import math

class GoalPosePublisher(Node):
    def __init__(self):
        super().__init__('goal_pose_publisher')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.path = self.read_path_from_file('src/webots_ros2_epuck/path.txt')
        self.stations = self.read_stations_from_file('src/webots_ros2_epuck/stations.txt')

    def read_path_from_file(self, filename):
        with open(filename, 'r') as file:
            return [int(station_id) for station_id in file.read().split() if station_id.isdigit()]

    def read_stations_from_file(self, filename):
        stations = {}
        with open(filename, 'r') as file:
            for line in file:
                parts = line.split(',')
                try:
                    station_id = int(parts[0].split(':')[1].strip())
                    x = float(parts[1].split(':')[1].strip())
                    y = float(parts[2].split(':')[1].strip())
                    stations[station_id] = {'x': x, 'y': y}
                except (ValueError, IndexError):
                    self.get_logger().warn(f"Skipping invalid line: {line}")
        return stations

    def is_in_goal_zone(self, current_x, current_y, goal_x, goal_y):
        return math.sqrt((current_x - goal_x)**2 + (current_y - goal_y)**2) < 0.3

    def odom_callback(self, msg):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        for station_id in self.path:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.position.x = self.stations[station_id]['x']
            goal_pose.pose.position.y = self.stations[station_id]['y']
            self.publisher.publish(goal_pose)
            
            timeout = time.time() + 5  # Увеличьте таймаут до 5 секунд
            while not self.is_in_goal_zone(current_x, current_y, goal_pose.pose.position.x, goal_pose.pose.position.y):
                rclpy.spin_once(self, timeout_sec=1)  # Ждем обновлений в топике /odom
                current_x = msg.pose.pose.position.x  # Обновляем текущее положение после спинов

        # Постим сообщение в топик /goal_pose с координатами (0, 0)
        final_goal_pose = PoseStamped()
        final_goal_pose.header.frame_id = 'map'
        final_goal_pose.pose.position.x = 0.0
        final_goal_pose.pose.position.y = 0.0
        self.publisher.publish(final_goal_pose)

        # Увеличиваем таймаут до 5 секунд
        timeout = time.time() + 5
        while not self.is_in_goal_zone(current_x, current_y, final_goal_pose.pose.position.x, final_goal_pose.pose.position.y):
            rclpy.spin_once(self, timeout_sec=1)
            current_x = msg.pose.pose.position.x  # Обновляем текущее положение после спинов

def main(args=None):
    rclpy.init(args=args)
    node = GoalPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
