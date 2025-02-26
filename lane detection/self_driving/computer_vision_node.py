import cv2
from geometry_msgs.msg import Twist
import datetime
from rclpy.node import Node 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
from .drive_BOT import Car
import rclpy
from std_msgs.msg import String

class Video_feed_in(Node):
    def __init__(self):
        super().__init__('video_subscriber')
        
        # Publishers
        self.publisher_img_read = self.create_publisher(Image, '/imgread', 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers - fixed overwriting issue
        self.image_subscriber = self.create_subscription(
            Image,
            '/color/video/image',
            self.process_data,
            10
        )
        self.traffic_sign_subscriber = self.create_subscription(
            String, 
            'traffic_sign', 
            self.traffic_sign_callback, 
            10
        )
        
        # Timer
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.send_cmd_vel)
        
        # Initialize other components
        self.velocity = Twist()
        self.bridge = CvBridge()
        self.Car = Car()
        self.p_err = 0
        self.vel = ''
        
        # Log node startup
        self.get_logger().info('Video feed node initialized')

    def traffic_sign_callback(self, msg):
        self.vel = msg.data
        self.get_logger().debug(f'Received traffic sign: {self.vel}')
        
    def send_cmd_vel(self):
        self.publisher.publish(self.velocity)
        
    def process_data(self, data): 
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            
            if frame is None:
                self.get_logger().warn('Received empty frame')
                return
                
            # Process frame
            self.p_err, Angle, Speed, img = self.Car.drive_car(frame, self.p_err, self.vel)

            # Update velocity commands
            self.velocity.angular.z = Angle
            self.velocity.linear.x = Speed    
            
            # Display processed image
            cv2.imshow("Frame", img)
            cv2.waitKey(1)

            # Publish processed image
            try:
                img_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
                self.publisher_img_read.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f'Error publishing processed image: {str(e)}')
                
        except Exception as e:
            self.get_logger().error(f'Error processing frame: {str(e)}')
        
def main(args=None):
    rclpy.init(args=args)
    
    try:
        image_subscriber = Video_feed_in()
        rclpy.spin(image_subscriber)
    except Exception as e:
        print(f'Error in main loop: {str(e)}')
    finally:
        # Cleanup
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()