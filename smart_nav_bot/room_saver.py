#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from smart_nav_bot.srv import LocationSave

class RoomSaver(Node):
    def __init__(self):
        super().__init__("room_saver")
        self.location_client = self.create_client(LocationSave, "save_location")
        self.save_rooms()

    def save_rooms(self):
        run = True
        
        while run:
            room_name = input("Enter room name or q to quit: ")
            if room_name != 'q':
                request = LocationSave.Request()
                request.room_name = room_name

                try:
                    x = float(input("Enter x coordinate: "))
                    y = float(input("Enter y coordinate: "))

                    request.pose.position.x = x
                    request.pose.position.y = y
                    request.pose.position.z = 0.0

                    request.pose.orientation.w = 1.0

                    if not self.location_client.wait_for_service(timeout_sec=3):
                        self.get_logger().info("Service unavailable")
                        return
                    
                    future = self.location_client.call_async(request)
                    rclpy.spin_until_future_complete(self, future)
                    response = future.result()

                    if response.success:
                        self.get_logger().info(room_name + " saved successfully")
                    
                    else:
                        self.get_logger().info("Room save failed")
                
                except Exception as e:
                    self.get_logger().info("Coordinates were entered wrong")
                    self.get_logger().info(e)
            
            else:
                run = False

def main(args=None):
    rclpy.init(args=args)
    node = RoomSaver()
    rclpy.shutdown()

if __name__ == "__main__":
    main()