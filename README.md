# 1257 Robot Code
This project is meant to be 1257's code, but upgraded with Path Planner and Advantage Kit.

This code includes the code for the drivetrain as well as the elevator subsystem. 
In the elevator subsystem folder, there are three files, respectively called Elevator.java, ElevatorIO.Java, ElevatorIO.sim. 

Project Structure: 
The project is structured as follows: 
├── src/main
│   ├── java
│   │   ├── frc
│   │   │   ├── robot
│   │   │   │   ├── subsystems
│   │   │   │   │   ├── drive
│   │   │   │   │   │   ├──Drive.java
│   │   │   │   │   │   ├──DriveIO.java
│   │   │   │   │   │   ├──DriveIOSim.java
│   │   │   │   │   │   ├──DriveIoSparkMax.java
│   │   │   │   │   │   ├──GyroIOReal.java
│   │   │   │   │   ├──elevator
│   │   │   │   │   │   ├──Elevator.java
│   │   │   │   │   │   ├──ElevatorIO.java
│   │   │   │   │   │   ├──ElevatorIOSim.java
│   │   │   │   │   │   ├──ElevatorIOSparkMax.java
│   │   │   ├── RobotContainer.java
├── ...
DriveSubsystem.java: Code for controlling the robot's drivetrain.
ElevatorSubsystem.java: Code for managing the elevator subsystem.
Drive Code
The drive code is responsible for controlling the robot's movement. It includes methods for driving forward, backward, turning, and strafing. The code uses the WPILib library for interacting with the hardware, such as motors and encoders.

Key features:
Elevator Subsystem Code
The elevator subsystem code manages the robot's elevator mechanism. It provides methods for controlling the elevator's height and position.

Key features:

Precise control of the elevator position.
Safety checks to prevent collisions.
Integration with limit switches and encoders.
Installation and Setup

Build and deploy the code to the robot.

Usage
The usage instructions depend on the specific features and controls implemented in the code. Refer to the relevant sections in DriveSubsystem.java and ElevatorSubsystem.java for details on how to control the robot.