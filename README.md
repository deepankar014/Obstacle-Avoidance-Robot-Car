🚗 Obstacle Avoidance Robot Car
An intelligent autonomous robot car designed to detect and avoid obstacles in its path using ultrasonic sensing and smart motor control. This project demonstrates the integration of embedded systems, sensors, and electromechanical components to create a self-navigating robotic vehicle.

📌 Project Overview
This Obstacle Avoidance Robot Car uses an Ultrasonic Sensor (HC-SR04) mounted on a servo motor to scan the surroundings and detect obstacles. Based on the measured distance, the Arduino Uno Rev 3 processes the data and controls the movement of the car through an L298N motor driver, enabling it to move forward, turn left, right, or stop to avoid collisions.

The robot continuously monitors its environment and intelligently changes direction when an obstacle is detected within a predefined range.

🎯 Features
Autonomous obstacle detection and avoidance
Real-time distance measurement
Smart decision-making for directional movement
Smooth turning using servo-based scanning
Modular and expandable design
Low-cost and beginner-friendly implementation
🧠 Working Principle
The ultrasonic sensor emits sound waves and measures the echo time to calculate distance.

The Arduino reads this distance value continuously.

If no obstacle is detected within the threshold distance, the car moves forward.

If an obstacle is detected:

The servo rotates the ultrasonic sensor to the left and right.
Distances in both directions are compared.
The car turns towards the direction with more free space.
The cycle repeats, enabling continuous navigation without human intervention.

🔧 Components Used
Core Components
Arduino Uno Rev 3
Ultrasonic Sensor (HC-SR04)
L298N Motor Driver Module
Servo Motor (SG90 / similar)
4 × DC Motors
4 × Wheels
Power Supply
9V Battery / 12V Battery Pack
On/Off Toggle Switch
Structural Components
Robot Car Chassis
Motor Mounts
Screws, Nuts, and Spacers
Additional Components
Jumper Wires (Male-Male, Male-Female)
Breadboard (optional)
Connecting Cables
Battery Holder
⚙️ Hardware Connections
Ultrasonic Sensor (HC-SR04)
VCC → 5V
GND → GND
Trig → Arduino Digital Pin
Echo → Arduino Digital Pin
Servo Motor
VCC → 5V
GND → GND
Signal → Arduino PWM Pin
L298N Motor Driver
IN1, IN2 → Left Motor Control Pins
IN3, IN4 → Right Motor Control Pins
ENA, ENB → PWM Pins (Speed Control)
12V → Battery Positive
GND → Battery Ground & Arduino GND
DC Motors
Connected to OUT1-OUT4 of L298N
🔁 Flow of Operation
Start the robot.
Move forward continuously.
Measure distance using ultrasonic sensor.
If distance > threshold → Move forward.
If distance < threshold → Stop and scan left/right.
Compare distances.
Turn towards the safer direction.
Resume forward movement.
💻 Software Requirements
Arduino IDE
Arduino Uno Board Package
Servo Library (built-in)
📂 Project Structure
Obstacle-Avoidance-Robot/
│
├── Code/
│   └── obstacle_avoidance.ino
├── Circuit Diagram/
│   └── wiring_diagram.png
├── Images/
│   └── robot_car.jpg
└── README.md
🚀 How to Run the Project
Connect all components as per the circuit diagram.
Upload the Arduino code using Arduino IDE.
Power the system using battery.
Place the robot on the floor.
Switch ON and observe autonomous navigation.
✅ Applications
Autonomous robotics learning
Obstacle detection systems
Smart vehicle prototypes
Robotics competitions
Educational demonstrations
🔮 Future Improvements
Add Bluetooth or WiFi control
Integrate camera module
Implement PID control for smoother movement
Add IR sensors for edge detection
GPS-based navigation
🧪 Project Outcome
The robot successfully avoids obstacles in real-time and changes direction dynamically, showcasing efficient integration of sensing, control, and motion systems. It serves as a practical demonstration of embedded system design and autonomous robotics.

👨‍💻 Developed By
Deepankar Majee

Electronics & Communication Engineering Student, GEC Gandhinagar

Focus Areas: Embedded systems, VLSI systems, Digital Design, and ASIC Flow

⭐ Acknowledgements
Special thanks to open-source Arduino community and robotics mentors for learning resources and guidance.
