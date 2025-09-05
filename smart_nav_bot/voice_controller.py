#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import speech_recognition as sr
import pyttsx3
from smart_nav_bot.srv import MoveToRoom, GetRoomName

class VoiceController(Node):
    def __init__(self):
        super().__init__("voice_controller")

        self.navigate_client = self.create_client(MoveToRoom, "navigate_room")
        self.get_room_client = self.create_client(GetRoomName, "get_room_names")

        self.recogniser = sr.Recognizer()
        self.microphone = sr.Microphone()

        self.engine = pyttsx3.init()
        voices = self.engine.getProperty('voices')
        self.engine.setProperty('voice', voices[1].id)
        self.engine.setProperty('rate', 160)
        self.engine.setProperty('volume', 0.8)

        self.get_logger().info("Say something")
        self.speak("Voice command is ready. Say a room name")

        self.listen_for_commands()
    
    def speak(self, text):
        self.engine.say(text)
        self.engine.runAndWait()

    def listen_for_commands(self):
        while rclpy.ok():
            try:
                with self.microphone as source:
                    self.recogniser.adjust_for_ambient_noise(source)
                    self.get_logger().info(f"Say something...")
                    audio = self.recogniser.listen(source, timeout=3, phrase_time_limit=5)


                command = self.recogniser.recognize_google(audio)
                self.get_logger().info(f"You said: {command}")
                self.process_commands(command.lower())
            except sr.UnknownValueError:
                self.get_logger().warn("Could not understand the command.")
                self.speak("I didn't understand that. Please try again.")
            except sr.RequestError as e:
                self.get_logger().error(f"Could not request results; {e}")
                self.speak("Service failed")
            except:
                self.get_logger().info("Didn't get a response, please try again")

    def process_commands(self, text):
        if not self.get_room_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("Service unavailable")
            self.speak("Navigation service is not available")
            return
        
        request = GetRoomName.Request()

        future = self.get_room_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        keywords = future.result().room_names

        room_found = None
        for keyword in keywords:
            if keyword in text:
                room_found = keyword

        if room_found:
            self.get_logger().info(f"We found room {room_found}")
            self.speak(f"Moving to {room_found}")
            self.call_navigation_service(room_found)
        else:
            self.get_logger().info(f"Room can't be found")
    
    def call_navigation_service(self, room_name):
        if not self.navigate_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("Service unavailable")
            self.speak("Navigation service is not available")
            return

        request = MoveToRoom.Request()
        request.room_name = room_name

        future = self.navigate_client.call_async(request)
        future.add_done_callback(self.navigation_response_callback)

    def navigation_response_callback(self, future):
        try:
            response = future.result()

            if response.success:
                self.get_logger().info(f"Navigation started: {response.msg}")
                self.speak("Navigation started")
            else:
                self.get_logger().info(f"Navigation unsuccessful: {response.msg}")
                self.speak("Failed to navigate")
        
        except Exception as e:
            self.get_logger().error(e)
            self.speak("Navigation error occurred")

def main(args=None):
    rclpy.init(args=args)
    node = VoiceController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()