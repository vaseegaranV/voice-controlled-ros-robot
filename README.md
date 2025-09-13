# Smart Nav Bot

A voice-controlled navigation system for ROS 2 robots that allows users to save room locations and navigate to them using voice commands. The system provides audio feedback through text-to-speech and integrates seamlessly with Nav2 for autonomous navigation.

## Features

- **Voice Control**: Navigate to rooms using natural voice commands
- **Room Management**: Save and retrieve room locations with coordinates
- **Audio Feedback**: Text-to-speech announces navigation status and results
- **Nav2 Integration**: Uses the Nav2 navigation stack for path planning and execution
- **Interactive Setup**: Easy room coordinate input through command-line interface

## System Architecture

The system consists of several interconnected ROS 2 nodes:

### Core Nodes

1. **Location Manager** (`location_manager`)
   - Stores room names and their corresponding poses
   - Provides services for saving locations and retrieving room data
   - Acts as an action client to communicate with the navigation system

2. **Navigation Manager** (`navigation_manager`) 
   - ROS 2 action server that handles navigation requests
   - Retrieves room coordinates and sends goals to Nav2
   - Provides navigation progress feedback and handles cancellations

3. **Voice Controller** (`voice_controller.py`)
   - Listens for voice commands using speech recognition
   - Processes commands to identify room names
   - Initiates navigation requests through service calls

4. **Result Subscriber** (`result_subscriber.py`)
   - Subscribes to navigation results
   - Announces outcomes using text-to-speech

### Utility Nodes

5. **Room Saver** (`room_saver.py`)
   - Interactive command-line tool for saving room coordinates
   - Allows users to input room names and x,y positions

## Prerequisites

### System Requirements
- ROS 2 (tested with Humble)
- TurtleBot3 packages
- Nav2 navigation stack
- Gazebo simulation environment

### Python Dependencies
```bash
pip install pyttsx3 SpeechRecognition pyaudio
```

### ROS 2 Dependencies
```bash
sudo apt install ros-humble-turtlebot3* ros-humble-navigation2 ros-humble-nav2-bringup
```

## Installation

1. Create a ROS 2 workspace and clone the repository:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <repository-url> smart_nav_bot
```

2. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select smart_nav_bot
source install/setup.bash
```

3. Ensure your map file is located at:
```
~/ros2_ws/src/smart_nav_bot/maps/my_house_map.yaml
```

## Usage

### 1. Launch the Simulation Environment

Start TurtleBot3 in Gazebo with Nav2:
```bash
ros2 launch smart_nav_bot simulation_launch.py
```

This will:
- Set up the TurtleBot3 burger model
- Launch Gazebo with the default world
- Start Nav2 with your custom map after a 10-second delay

### 2. Launch the Smart Nav Bot System

In a new terminal, start all navigation nodes:
```bash
ros2 launch smart_nav_bot smart_nav_launch.py
```

This launches:
- Location Manager
- Navigation Manager  
- Voice Controller
- Result Subscriber

### 3. Save Room Locations

Use the room saver utility to define room coordinates:
```bash
ros2 run smart_nav_bot room_saver.py
```

Follow the prompts to enter room names and their x,y coordinates. Type 'q' to quit.

### 4. Voice Navigation

Once rooms are saved, simply speak room names to navigate:
- The system will listen for voice commands
- Say a room name (e.g., "kitchen", "bedroom")
- The robot will navigate to the specified location
- Audio feedback will announce the navigation status

## Services and Actions

### Services

- `/save_location` (`smart_nav_bot/srv/LocationSave`)
  - Save a room name with its pose coordinates

- `/navigate_room` (`smart_nav_bot/srv/MoveToRoom`) 
  - Trigger navigation to a specified room

- `/get_room_names` (`smart_nav_bot/srv/GetRoomName`)
  - Retrieve all saved room names and poses

### Actions

- `/navigate_to_room` (`smart_nav_bot/action/NavigateToRoom`)
  - Navigate to room action with progress feedback

### Topics

- `/nav_goal_result` (`std_msgs/msg/String`)
  - Navigation result messages for TTS feedback

## Configuration

### Voice Recognition Settings
- Microphone adjustment for ambient noise
- 3-second timeout for voice input
- 5-second phrase time limit

### Text-to-Speech Settings
- Voice selection (uses second available voice)
- Speech rate: 160 words per minute
- Volume: 0.8

### Navigation Settings
- Frame ID: "map"
- Service timeout: 5 seconds
- Feedback update rate: 100ms

## Troubleshooting

### Common Issues

1. **Voice recognition not working**
   - Check microphone permissions
   - Ensure pyaudio is properly installed
   - Verify internet connection for Google Speech Recognition

2. **Navigation fails**
   - Confirm Nav2 is running and map is loaded
   - Check that room coordinates are within map bounds
   - Verify TurtleBot3 model is set correctly

3. **TTS not working**
   - Install espeak: `sudo apt install espeak`
   - Check audio output settings

### Service Debugging

Check if services are available:
```bash
ros2 service list
```

Monitor navigation feedback:
```bash
ros2 topic echo /nav_goal_result
```

## File Structure

```
smart_nav_bot/
├── src/
│   ├── navigation_manager.cpp      # Navigation action server
│   └── location_manager.cpp        # Room storage and management
├── scripts/
│   ├── voice_controller.py         # Voice command processing
│   ├── result_subscriber.py        # TTS result announcements
│   └── room_saver.py              # Interactive room coordinate input
├── launch/
│   ├── simulation_launch.py        # Gazebo and Nav2 launch
│   └── smart_nav_launch.py        # Smart nav bot nodes launch
├── maps/
│   └── my_house_map.yaml          # Navigation map
└── README.md
```
- Uses TurtleBot3 simulation environment
- Speech recognition powered by Google Speech API
- Text-to-speech using pyttsx3
