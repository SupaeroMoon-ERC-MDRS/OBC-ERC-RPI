#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from .arm_controller import ArmController
import numpy as np

def main(args=None):
    rclpy.init(args=args)
    
    # Create arm controller instance
    arm_controller = ArmController()
    
    try:
        # Example: Move to a specific position
        position = [0.3, 0.0, 0.2]  # x, y, z in meters
        orientation = [0.0, np.pi/2, 0.0]  # roll, pitch, yaw in radians
        
        # Move the arm
        success = arm_controller.move_to_position(position, orientation)
        
        if success:
            arm_controller.get_logger().info('Movement completed successfully')
        else:
            arm_controller.get_logger().error('Movement failed')
            
        # Keep the node running
        rclpy.spin(arm_controller)
            
    except KeyboardInterrupt:
        pass
    finally:
        arm_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 