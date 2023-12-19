from rclpy.node import Node
import rclpy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

    # 获取 local_costmap 数据，并进行DWA计算
    # 实现DWA局部路径规划算法

    # 这里需要你实现DWA局部路径规划的代码
    # 计算机器人应该移动的线速度和角速度
    # 更新 self.twist.linear.x 和 self.twist.angular.z

    # 将计算出的速度发布到/cmd_vel话题

class LocalController(Node):
    def __init__(self):
        super().__init__('dwa_controller')
        self.costmap_sub = self.create_subscription('/local_costmap/costmap', OccupancyGrid, self.costmap_callback)
        self.odom_sub = self.create_subscription('/odom', Odometry, self.odom_callback)
        self.cmd_vel_pub = self.create_publisher('/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()
        self.obstacle_list = []
        
        self.v_min = -0.1
        self.v_max = 0.3
        self.w_min = -50 / 180.0 * np.pi
        self.w_max = 50 / 180.0 * np.pi
        self.v_a = 0.05
        self.w_a = 30 / 180.0 * np.pi
        self.v_resolution = 0.01
        self.w_resolution = 1 / 180.0 * np.pi
        self.robot_r = 0.2
        self.dt = 0.05
        self.predict_time = 1
        self.alpha_goal_coef = 1
        self.beta_velocity_coef = 1
        self.gamma_obstacle_coef = 1

    def costmap_callback(self, msg):
        self.localmap_width = msg.info.width
        self.localmap_height = msg.info.height
        self.localmap_resolution = msg.info.resolution
        self.localmap_origin_x = msg.info.origin.position.x
        self.localmap_origin_y = msg.info.origin.position.y
        
        self.costmap = msg.data
        self.get_obstacles()
        
    def get_obstacles(self):
        for y in range(self.localmap_height):
            for x in range(self.localmap_width):
                index = x + y * self.localmap_width
                if self.costmap[index] != 0:  # Occupied cell
                    # Convert map index to world coordinates
                    world_x = self.localmap_origin_x + (x + 0.5) * self.localmap_resolution
                    world_y = self.localmap_origin_y + (y + 0.5) * self.localmap_resolution
                    self.obstacle_list.append([world_x, world_y])

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.angle = msg.pose.pose.orientation.w
        self.vx = msg.twist.linear.x
        self.vy = msg.twist.linear.y
        self.v = np.sqrt(self.vx**2 + self.vy**2)
        self.w = msg.twist.angular.z
        
    def get_goal(self, goal):
        self.goal_pos = [goal.pose.position.x, goal.pose.position.y, 
                         goal.pose.orientation.z, goal.pose.orientation.w]
        
    def Goal_Cost(self, poses, goal):
        return np.sqrt((poses[-1, 1] - self.goal_pos[0])**2 + (self.y - self.goal_pos[1])**2)
    
    def Velocity_Cost(self):
        pass
        
    def Obstacle_Cost(self, poses, obstacles):
        min_distance = float('Inf')
        for i in range(len(poses)):
            for j in range(len(obstacles)):
                current_distance = np.sqrt((poses[i,0] - obstacles[j,0])**2 + (poses[i,1] - obstacles[j,1])**2)
                if current_distance <= self.robot_r:
                    return float('Inf')
                if current_distance < min_distance:
                    min_distance = current_distance
        return 1 / min_distance   
        
    def vw_range(self):
        v_min_actual = max(self.v_min, self.v - self.dt * self.v_a)
        v_max_actual = max(self.v_max, self.v + self.dt * self.v_a)
        w_min_actual = max(self.w_min, self.w - self.dt * self.w_a)
        w_max_actual = max(self.w_max, self.w + self.dt * self.w_a)
        return [v_min_actual, v_max_actual, w_min_actual, w_max_actual]
        
    def motion(self, x, u, dt):
        x[0] += u[0] * dt * np.cos(x[2])
        x[1] += u[0] * dt * np.sin(x[2])
        x[2] += u[1] * dt
        x[3] = u[0]
        x[4] = u[1]
        return x
        
    def Trajectory_Calculate(self, x, u):
        Trajectory = np.array(x)
        x_new = np.array(x)
        time = 0
        while time <= self.predict_time:
            x_new = self.motion(x_new, u, self.dt)
            Trajectory = np.vstack((Trajectory, x_new))
            time += self.dt
        return Trajectory
        
    def DWAcontroller(self, x, ):
        vw_limit = self.vw_range()
        
        
        
        
        
        
        
        
        

        self.cmd_vel_pub.publish(self.twist)
        
        
# if __name__ == '__main__':
#     try:
#         controller = LocalController()
#         rclpy.spin()
#     except rclpy.ROSInterruptException:
#         pass