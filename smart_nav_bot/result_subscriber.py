#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyttsx3

class ResultSubscriber(Node):
    def __init__(self):
        super().__init__("result_subscriber")

        self.nav_goal_result_sub = self.create_subscription(String, "nav_goal_result", self.nav_goal_result_callback, 10)

        self.engine = pyttsx3.init()
        voices = self.engine.getProperty('voices')
        self.engine.setProperty('voice', voices[1].id)
        self.engine.setProperty('rate', 160)
        self.engine.setProperty('volume', 0.8)
    
    def speak(self, text):
        self.engine.say(text)
        self.engine.runAndWait()

    def nav_goal_result_callback(self, msg):
        self.get_logger().info("Saying result")
        self.speak(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = ResultSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()