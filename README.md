# Differential-drive-robot-simulator-in-ROS2


### Description of the Overall Approach:
**Overview and Purpose:**
The goal of this project is to create a simulator for a differential drive robot using ROS (Robot Operating System). The simulation involves the movement of the robot, considering parameters such as wheel distance, error variance in wheel velocities, and error update rate. The simulation involves two nodes: the Simulator Node and the Velocity Translator Node. The former simulates
the robot's movement based on velocity commands, while the latter translates linear and angular velocities into wheel velocities.

**Conceptual Approach:**
The conceptual approach involves creating two nodes that work together to simulate the movement of a differential drive robot. The Velocity Translator Node simplifies control by translating linear and angular velocities into left and right wheel velocities. The Simulator Node utilizes these wheel velocities to update the robot's pose, considering simulated errors.

**Node 1: Velocity Translator Node**
The Velocity Translator Node is responsible for converting linear and angular velocities into left and right wheel velocities. The node subscribes to the /cmd_vel topic and publishes on /vl and /vr. The implementation uses the standard ROS2 Python library and the geometry_msgs and std_msgs message types.
The node's primary functionalities include:

1. Initialization of parameters, such as wheel distance (L).
2. Subscription to /cmd_vel topic.
3. Calculation of left and right wheel velocities based on received linear and angular velocities.
4. Logging of wheel velocities for debugging purposes.
5. Publishing the calculated wheel velocities on /vl and /vr topics.
6. This node simplifies the control of the simulated robot.

**Node 2: Simulator Node**
The Simulator Node is responsible for simulating the movement of the differential drive robot. It subscribes to /vl and /vr topics, updates the robot's pose based on received wheel velocities, and broadcasts tf frames for visualization. The node also introduces simulated errors in wheel velocities.
Key features of the Simulator Node include:

1. Initialization of parameters, including wheel distance, error variances, and error update rate.
2. Subscription to /vl and /vr topics.
3. Update of simulated errors in wheel velocities at a specified rate.
4. Logging of velocity and error information for debugging.
5. Calculation of the robot's new pose based on received wheel velocities.
6. Broadcasting tf frames for visualization in rviz.

### Launch File and System Integration
To ensure synchronized operation, a launch file is used to initialize the system. It sets up the necessary arguments for robot file, input and output bag files, starts the playback of recorded left wheel velocities and right wheel velocities, initiates all the required nodes, and captures the system's output to the specified output bag file. Choices Made in Designing the Program and Rationale

**1. Parameterization for Flexibility:**
Decision: The program utilizes parameterization to enhance flexibility, allowing dynamic adjustments to key parameters such as wheel distance, error variances, and update rates.
Rationale: Parameterization ensures adaptability to different robot configurations and error models without the need for code modifications. This design choice facilitates experimentation with diverse scenarios, promoting a more versatile and user-friendly simulation environment.

**2. Condition on /vl and /vr in Update Pose:**
Decision: An if-else condition is implemented in the update pose function to prevent division by zero when the left wheel velocity is equal to the right wheel velocity.
Rationale: This conditional check is crucial to avoid mathematical errors and potential runtime crashes. By ensuring that the division operation is valid, the program maintains numerical stability,
contributing to the robustness of the simulator.

**3. Wrapping of Heading (self.theta):**
Decision: The program incorporates heading wrapping to constrain self.theta within the range of -6.28 to +6.28 radians.
Rationale: Wrapping the heading ensures continuous representation of angular orientation, preventing unbounded growth or sudden jumps in orientation values. This decision promotes numerical stability and consistency in tracking the robot's orientation throughout its movement.

**4. Use of sys.argv to Get Robot File Name:**
Decision: The launch file employs sys.argv to directly obtain the robot file name from the console, as direct concatenation with the launch configuration object was not feasible.
Rationale: Utilizing sys.argv allows for seamless retrieval of the robot file name directly from the command line, addressing limitations in concatenating addresses with launch configuration objects. This decision enhances user convenience during execution, making it easier to specify different robot configurations without modifying the launch file.

**5. Use of a Launch File for System Integration:**
Decision: The program is designed with a launch file to orchestrate the integration of various nodes, processes, and configurations required for the differential drive robot simulator.
Rationale: A launch file serves as a centralized and modular orchestrator, streamlining the integration of different components. This design choice promotes maintainability, scalability, and ease of configuration. It enables efficient testing of diverse scenarios and promotes a systematic approach to system integration, contributing to a more robust and user-friendly simulation
environment.

**Did the results meet our expectations?**

The implementation adheres to the specified requirements, incorporating key features such as parameterization for flexibility, robust error handling, and continuous orientation representation. The use of a launch file for system integration enhances maintainability and scalability. Additionally, the simulator successfully simulates the movement of a differential drive robot, and the velocity translator node effectively translates linear and angular velocities into appropriate wheel velocities. Overall, the system aligns with the outlined project goals, demonstrating a well-structured and functional differential drive robot simulator.

## Results:

![image](https://github.com/khullarsanket/Differential-drive-robot-simulator-in-ROS2/assets/119709438/0647e4c5-f448-4506-a1c7-0f40a6703f2e)


![image](https://github.com/khullarsanket/Differential-drive-robot-simulator-in-ROS2/assets/119709438/4d24fd0b-da2a-4d75-b118-a2265f67e641)


![image](https://github.com/khullarsanket/Differential-drive-robot-simulator-in-ROS2/assets/119709438/aa5fb1f2-33c1-4bba-94a5-781d02d96dfa)


![image](https://github.com/khullarsanket/Differential-drive-robot-simulator-in-ROS2/assets/119709438/a10512b9-45c6-47c4-8a0c-38e9cc06c3c0)


![image](https://github.com/khullarsanket/Differential-drive-robot-simulator-in-ROS2/assets/119709438/b45f3107-b346-408d-be76-e9db0547ff07)

