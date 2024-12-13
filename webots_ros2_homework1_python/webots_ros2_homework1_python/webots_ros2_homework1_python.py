
import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
# import Quality of Service library, to set the correct profile and reliability in order to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math





LINEAR_VEL = 0.22
STOP_DISTANCE = 0.1
LIDAR_ERROR = 0.05
LIDAR_AVOID_DISTANCE = 0.605
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
RIGHT_SIDE_INDEX = 315
RIGHT_FRONT_INDEX = 220
LEFT_FRONT_INDEX=150
LEFT_SIDE_INDEX=45
MAX_WALL_DIST=0.280
MIN_WALL_DIST=0.270


class RandomWalk(Node):


    def __init__(self):
        # Initialize the publisher
        super().__init__('random_walk_node')
        self.scan_cleaned = []
        self.positions = []
        self.stall = False
        self.turtlebot_moving = False
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber1 = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscriber2 = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback2,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.laser_forward = 0
        self.odom_data = 0
        timer_period = 0.5
        self.pose_saved=''
        self.cmd = Twist()
        self.timer = self.create_timer(timer_period, self.timer_callback)




    def listener_callback1(self, msg1):
        #self.get_logger().info('scan: "%s"' % msg1.ranges)
        scan = msg1.ranges
        self.scan_cleaned = [3.5 if reading == float('Inf') else (0.0 if math.isnan(reading) else reading) for reading in scan]






    def listener_callback2(self, msg2):
        position = msg2.pose.pose.position
        self.get_logger().info(f'Self position: {position.x}, {position.y}, {position.z}')
        # similarly for twist message if you need
        self.pose_saved=position
       
        #Example of how to identify a stall..need better tuned position deltas; wheels spin and example fast
        #diffX = math.fabs(self.pose_saved.x- position.x)
        #diffY = math.fabs(self.pose_saved.y - position.y)
        #if (diffX < 0.0001 and diffY < 0.0001):
           #self.stall = True
        #else:
           #self.stall = False
           
        return None
       
    def timer_callback(self):
        if (len(self.scan_cleaned)==0):
            self.turtlebot_moving = False
            return
           
        #left_lidar_samples = self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX]
        #right_lidar_samples = self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX]
        #front_lidar_samples = self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX]
       
        left_lidar_min = min(self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX])
        right_lidar_min = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])
        front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX])
        straight_ahead_min = min(self.scan_cleaned[(LEFT_FRONT_INDEX):(RIGHT_FRONT_INDEX-30)])
        front_left_min = min(self.scan_cleaned[135:(LEFT_FRONT_INDEX+15)])


        #self.get_logger().info('left scan slice: "%s"'%  min(left_lidar_samples))
        #self.get_logger().info('front scan slice: "%s"'%  min(front_lidar_samples))
        #self.get_logger().info('right scan slice: "%s"'%  min(right_lidar_samples))
       
        if front_lidar_min < SAFE_STOP_DISTANCE:
            if self.turtlebot_moving == True:
                self.cmd.linear.x = -0.1
                self.cmd.angular.z = 0.0
                self.publisher_.publish(self.cmd)
                self.turtlebot_moving = False
                self.get_logger().info('reversing')
                return
        elif front_lidar_min< LIDAR_AVOID_DISTANCE:
            if straight_ahead_min > (2*LIDAR_AVOID_DISTANCE):
                self.cmd.linear.x = 0.3
                self.cmd.angular.z = 0.0
                self.publisher_.publish(self.cmd)
                self.get_logger().info('Tunnel ahead')
                self.turtlebot_moving = True
            else:
                self.cmd.linear.x = 0.07
                self.cmd.angular.z = -0.5
                self.publisher_.publish(self.cmd)
                self.get_logger().info('Hard Turning Right')
        elif front_left_min < (((LIDAR_AVOID_DISTANCE+MIN_WALL_DIST)/2)+0.05):
                self.cmd.linear.x = 0.07
                self.cmd.angular.z = -0.3
                self.publisher_.publish(self.cmd)
                self.get_logger().info('Hard Turning Right FL')
        elif left_lidar_min<MIN_WALL_DIST:
            if left_lidar_min<(MIN_WALL_DIST-0.05):
                self.cmd.linear.x = 0.07
                self.cmd.angular.z = -0.3
                self.publisher_.publish(self.cmd)
                self.get_logger().info('Hard Turning Right')
            else:
                self.cmd.linear.x = 0.12
                self.cmd.angular.z = -0.2
                self.publisher_.publish(self.cmd)
                self.get_logger().info('Turning Right')
            self.turtlebot_moving = True
        elif left_lidar_min>MAX_WALL_DIST:
            if front_lidar_min<left_lidar_min and front_lidar_min>LIDAR_AVOID_DISTANCE:
                self.cmd.linear.x = 0.3
                self.cmd.angular.z = 0.0
                self.publisher_.publish(self.cmd)
                self.get_logger().info('Finding Wall')
                self.turtlebot_moving = True
            elif front_lidar_min<left_lidar_min and front_lidar_min<LIDAR_AVOID_DISTANCE:
                self.cmd.linear.x = 0.05
                self.cmd.angular.z = -0.4
                self.publisher_.publish(self.cmd)
                self.get_logger().info('Finding Wall')
                self.turtlebot_moving = True
            elif front_lidar_min>(MAX_WALL_DIST+0.05):
                self.cmd.linear.x = 0.07
                self.cmd.angular.z = 0.3
                self.publisher_.publish(self.cmd)
                self.get_logger().info('Hard Turning Left')
                self.turtlebot_moving = True

            else:
                self.cmd.linear.x = 0.12
                self.cmd.angular.z = 0.2
                self.publisher_.publish(self.cmd)
                self.get_logger().info('Turning Left')
                self.turtlebot_moving = True
        else:
            self.cmd.linear.x = 0.3
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = True


        #if front_lidar_min < SAFE_STOP_DISTANCE:
        #    if self.turtlebot_moving == True:
        #        self.cmd.linear.x = 0.0
        #        self.cmd.angular.z = 0.0
        #        self.publisher_.publish(self.cmd)
        #        self.turtlebot_moving = False
        #        self.get_logger().info('Stopping')
        #        return
        #elif front_lidar_min < LIDAR_AVOID_DISTANCE:
        #        self.cmd.linear.x = 0.07
        #        if (right_lidar_min > left_lidar_min):
        #           self.cmd.angular.z = -0.3
        #        else:
        #           self.cmd.angular.z = 0.3
        #        self.publisher_.publish(self.cmd)
        #        self.get_logger().info('Turning')
        #        self.turtlebot_moving = True
        #else:
        #    self.cmd.linear.x = 0.3
        #    self.cmd.linear.z = 0.0
        #    self.publisher_.publish(self.cmd)
        #    self.turtlebot_moving = True
           


        self.get_logger().info('Distance of the obstacle : %f' % front_lidar_min)
        self.get_logger().info('I receive: "%s"' %
                               str(self.odom_data))
        if self.stall == True:
           self.get_logger().info('Stall reported')
       
        # Display the message on the console
        self.get_logger().info('Publishing: "%s"' % self.cmd)





def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    random_walk_node = RandomWalk()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(random_walk_node)
    # Explicity destroy the node
    random_walk_node.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()






if __name__ == '__main__':
    main()
