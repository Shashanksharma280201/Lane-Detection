import cv2
from .Detection.Lanes.lane_detection import detect_lanes
from numpy import interp
import os
import numpy as np
from pathlib import Path

class Control:
    def __init__(self):
        # Initialize steering and speed control variables
        self.angle = 0.0
        self.speed = 0.36  # Default linear speed from ROS parameter
        
        # PID controller parameters
        self.pid = [0.005, 0.0, 0.005]  # [Kp, Ki, Kd]
        self.setpoint = 0
        self.p_err = 0
        self.i_err = 0
        self.i_err_max = 100  # Anti-windup limit
        self.text = ""
        self.alpha = 0.2  # Filter coefficient for derivative
        self.p_Ud = 0
        
        # Speed parameters
        self.max_linear_speed = 0.36  # Maximum linear speed (m/s)
        self.max_angular_speed = 0.5  # Maximum angular speed (rad/s)
        
        # Setup angle logging file
        self.pkg_dir = Path(os.path.dirname(__file__))
        self.angle_file = self.pkg_dir / 'angle.txt'
        self.angle_file.parent.mkdir(parents=True, exist_ok=True)
        
    def follow_lane(self, max_sane_dist, dist, curv):
        # Base speed setting
        self.speed = self.max_linear_speed
        
        # Speed adjustments based on traffic signs and road conditions
        if self.text == "60":
            self.speed = self.max_linear_speed * 0.75  # 75% of max speed
        elif self.text == "stop" or self.text == "red_light":
            self.speed = 0.0
        elif self.text == "30" or self.text == "green_light":
            self.speed = self.max_linear_speed * 0.5  # 50% of max speed
        elif curv > 30 or curv < -30 or dist < -200:
            self.speed = self.max_linear_speed * 0.6  # 60% of max speed for curves

        # Steering angle calculation
        max_turn_angle = 90
        max_turn_angle_neg = -90
        
        # Calculate required turn angle based on distance and curvature
        if dist > max_sane_dist:
            req_turn_angle = max_turn_angle
        elif dist < -max_sane_dist:
            req_turn_angle = max_turn_angle_neg
        else:
            req_turn_angle = interp(dist, [-max_sane_dist, max_sane_dist], 
                                  [-max_turn_angle, max_turn_angle])
        
        # Add curvature compensation
        req_turn_angle += curv
        
        # Limit turn angle to safe range
        req_turn_angle = np.clip(req_turn_angle, max_turn_angle_neg, max_turn_angle)
        
        # Map to final steering angle range
        self.angle = interp(req_turn_angle, [max_turn_angle_neg, max_turn_angle], [-45, 45])
        
    def drive(self, current_state):
        try:
            # Unpack current state
            dist, curv, img = current_state

            # Check for valid data
            if dist != 1000 and curv != 1000:
                self.follow_lane(img.shape[1] / 4, dist, curv)
            else:
                self.speed = 0.0
                print('Invalid lane detection data')
            
            # Log steering angle
            try:
                with open(self.angle_file, 'a') as f:
                    f.write(f"{self.angle}\n")
            except IOError as e:
                print(f"Warning: Could not log angle data: {e}")

            # PID control calculation
            self.err = self.setpoint - self.angle
            
            # Integral term with anti-windup
            self.i_err = np.clip(self.i_err + self.err, -self.i_err_max, self.i_err_max)
            
            # Filtered derivative term
            d_err = (1 - self.alpha) * (self.err - self.p_err) + self.alpha * self.p_Ud
            self.p_Ud = d_err
            
            # Calculate control output
            self.yaw = (self.pid[0] * self.err + 
                       self.pid[1] * self.i_err + 
                       self.pid[2] * d_err)
            
            # Clip yaw to max angular speed
            self.yaw = np.clip(self.yaw, -self.max_angular_speed, self.max_angular_speed)
            
        except Exception as e:
            print(f"Error in drive control: {e}")
            self.speed = 0.0
            self.yaw = 0.0

class Car:
    def __init__(self):
        self.control = Control()

    def update_parameters(self, linear_speed, angular_speed):
        """Update speed parameters from ROS parameters"""
        self.control.max_linear_speed = linear_speed
        self.control.max_angular_speed = angular_speed

    def display_state(self, frame_disp, angle_of_car, current_speed, text):
        # Map control values to display values
        angle_of_car = interp(angle_of_car, 
                            [-self.control.max_angular_speed, self.control.max_angular_speed], 
                            [45, -45])
        
        if current_speed != 0.0:
            current_speed = interp(current_speed, 
                                 [0, self.control.max_linear_speed], 
                                 [0, 90])

        # Determine direction and color
        if angle_of_car < -10:
            direction_string = "[ Left ]"
            color_direction = (120, 0, 255)
        elif angle_of_car > 10:
            direction_string = "[ Right ]"
            color_direction = (120, 0, 255)
        else:
            direction_string = "[ Straight ]"
            color_direction = (0, 255, 0)

        if current_speed > 0:
            direction_string = "Moving --> " + direction_string
        else:
            color_direction = (0, 0, 255)

        # Draw status information on frame
        cv2.putText(frame_disp, str(direction_string), (20, 40), 
                   cv2.FONT_HERSHEY_DUPLEX, 0.4, color_direction, 1)
        
        angle_speed_str = f"[ Angle, Speed ] = [ {int(angle_of_car)}deg, {current_speed:.2f}m/s ]"
        cv2.putText(frame_disp, str(angle_speed_str), (20, 20), 
                   cv2.FONT_HERSHEY_DUPLEX, 0.4, (0, 0, 255), 1)
        
        cv2.putText(frame_disp, str(text), (20, 60), 
                   cv2.FONT_HERSHEY_DUPLEX, 0.4, (0, 0, 255), 1)

    def drive_car(self, frame, p_err, text):
        try:
            self.control.p_err = p_err
            self.control.text = text

            # Extract region of interest and resize
            # img = frame[100:400, 40:600]
            img = cv2.resize(frame, (320, 240))

            # Detect lanes
            distance, curvature = detect_lanes(img)

            # Update control
            current_state = [distance, curvature, img]
            self.control.drive(current_state)

            # Display current state
            self.display_state(img, self.control.yaw, self.control.speed, text)

            return self.control.err, self.control.yaw, self.control.speed, img

        except Exception as e:
            print(f"Error in drive_car: {e}")
            return 0.0, 0.0, 0.0, frame