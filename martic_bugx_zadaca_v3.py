import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
import tf_transformations as transform

zero = 0.0


class Bugx(Node):
    
    #Inicijalizacija
    def __init__(self):
        super().__init__('bugx')
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.location_callback, 10)
        self.fl_sensor_sub = self.create_subscription(Range, '/fl_range_sensor', self.fl_sensor_callback, 10)
        self.fr_sensor_sub = self.create_subscription(Range, '/fr_range_sensor', self.fr_sensor_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.bug_algorithm_timer = self.create_timer(0.1, self.bug_algorithm_callback)
        self.current_x = zero
        self.current_y = zero
        self.goal_x = zero
        self.goal_y = zero
        self.current_theta = zero
        self.fl_sensor_value = zero
        self.fr_sensor_value = zero
        self.cmd_vel_msg = Twist()
        self.hit_point = zero
        self.leave_point = zero
        self.initial_position = zero
        self.final_position = zero
        self.direction_found = False
        self.state = 'pronadi_smijer'
        
    #Metoda ažuriranja cilja
    def goal_callback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.final_position = msg.pose.position
        
        
    #Metoda za trenutnu poziciju robota
    def location_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_position = msg.pose.pose.position
        q = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
            
        self.current_theta = transform.euler_from_quaternion(q)[2] #[-pi, pi]
        if self.initial_position is None:
            self.initial_position = self.current_position
            
            
    #Metode za ažuriranje vrijednosti senzora
    def fl_sensor_callback(self, msg):
        self.fl_sensor_value = msg.range / 0.70  
        if self.fl_sensor_value < zero:
            self.fl_sensor_value = zero

    def fr_sensor_callback(self, msg):
        self.fr_sensor_value = msg.range / 0.70 
        if self.fr_sensor_value < zero:
            self.fr_sensor_value = zero

    def calculate_euclidean_distance(self, a, c, b, d):
        return math.sqrt((a - b) ** 2 + (c - d) ** 2)

    def is_on_line(self):
        ciljani_smijer = [self.goal_x - self.initial_position.x, self.goal_y - self.initial_position.y]
        pozicija_vektora = [self.current_x - self.initial_position.x, self.current_y - self.initial_position.y]
        rezultat = ciljani_smijer[0] * pozicija_vektora[1] - ciljani_smijer[1] * pozicija_vektora[0]
        return abs(rezultat) < 0.01  

    def bug_algorithm_callback(self):
        if self.goal_x is None or self.goal_y is None:
            self.cmd_vel_msg.linear.x = zero
            self.cmd_vel_msg.angular.z = zero
            self.cmd_pub.publish(self.cmd_vel_msg)
            return

        if self.state == 'pronadi_smijer':
            ciljani_kut = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
            razlika_kuta = ciljani_kut - self.current_theta
            if razlika_kuta > math.pi:
                razlika_kuta -= 2 * math.pi
            elif razlika_kuta < -math.pi:
                razlika_kuta += 2 * math.pi
            self.cmd_vel_msg.angular.z = 0.3 * razlika_kuta
            self.cmd_vel_msg.linear.x = zero
            if abs(razlika_kuta) < 0.04:
                self.direction_found = True
                self.state = 'kreni_prema_cilju'

        elif self.state == 'kreni_prema_cilju':
            if not self.direction_found:
                return
            if self.fl_sensor_value < 0.7 or self.fr_sensor_value < 0.7:
                self.hit_point = (self.current_x, self.current_y)
                self.state = 'prati_granice'
                self.cmd_vel_msg.angular.z = 0.1 
                self.cmd_vel_msg.linear.x = zero
            else:
                ciljani_kut = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
                razlika_kuta = ciljani_kut - self.current_theta
                if razlika_kuta > math.pi:
                    razlika_kuta -= 2 * math.pi
                elif razlika_kuta < -math.pi:
                    razlika_kuta += 2 * math.pi
                self.cmd_vel_msg.angular.z = 0.3 * razlika_kuta  
                self.cmd_vel_msg.linear.x = 0.1  


        elif self.state == 'prati_granice':
            if self.fl_sensor_value > 0.7 and self.fr_sensor_value > 0.7:
                self.state = 'kreni_prema_cilju'
                self.leave_point = (self.current_x, self.current_y)
                self.cmd_vel_msg.angular.z = zero
                self.cmd_vel_msg.linear.x = 0.1

        elif self.state == 'prati_granice':
            if self.fl_sensor_value > 0.7 and self.fr_sensor_value > 0.7:
                self.state = 'kreni_prema_cilju'
                self.leave_point = (self.current_x, self.current_y)
                self.cmd_vel_msg.angular.z = zero
                self.cmd_vel_msg.linear.x = 0.1
            else:
                self.cmd_vel_msg.angular.z = 0.1
                self.cmd_vel_msg.linear.x = zero
                if self.is_on_line() and self.calculate_euclidean_distance(self.current_x, self.current_y, self.goal_x, self.goal_y) < 0.1: 
                    self.state = 'kreni_prema_cilju'
                    self.cmd_vel_msg.angular.z = zero
                    self.cmd_vel_msg.linear.x = zero

        self.cmd_pub.publish(self.cmd_vel_msg)
        

def main(args=None):

    rclpy.init(args=args)
    bug_node = Bugx()
    rclpy.spin(bug_node)
    bug_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()




