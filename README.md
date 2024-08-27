# Path-Reuse-Method

## 1. Introduction

This document provides an explanation of the code for the new path planning method developed by the Nishida Laboratory.

**Path-Reuse-Method** encodes information from various paths and stores it in the form of "path seeds," which can then be decoded and reused according to the task at hand. Unlike traditional methods that utilize stored primitives, the PR method allows for the expansion and reuse of paths between arbitrary start and goal positions. Additionally, by reusing paths with the desired shape, the number of collision checks in the T-space can be significantly reduced, leading to a substantial decrease in the overall computational cost of path generation.

## 2. Operating Environment

- OS: Ubuntu 20.04 LTS
- ROS Noetic
- CPU: Intel Core i7-12700
- Memory: 32 GB

## 3. Preparations before using our method

### 3.1 Install ROS Noetic

Please install ROS Noetic on Ubuntu 20.04 LTS.

References: https://wiki.ros.org/noetic/Installation/Ubuntu

### 3.2 Setting Up the xArm6 Environment

This planning method is adaptable to the 6-axis robot arm xArm6 provided by uFactory. Therefore, it is necessary to set up the operating environment for xArm6.

You can smoothly set up the environment by using the following repository we have prepared.

### 3.3 Setting Up the STOMP Environment

This method utilizes one of the trajectory optimization algorithms known as Stochastic Trajectory Optimization for Motion Planning (STOMP).

You can smoothly set up the environment by using the following repository we have prepared.

## 4. Generate new path through encoding and decoding

### 4.1 Download this repository

```
git clone https://github.com/kitaq-nishidalab/Path-Reuse-Method.git
```

Move to the encode&decode folder.

```
cd encode&decode
```

### 4.2 Save the path information

As a preliminary step, it is necessary to save the path information. The path information consists of a list of joint angles of the robot arm, which needs to be saved in a text file.

An example of the text file is shown below:

```
['2.73e-05', '-1.48e-01', '-9.87e-01', '1.25e-05', '1.14', '1.66e-06']
['2.67e-05', '-1.55e-01', '-1.03', '1.20e-05', '1.18', '2.04e-06']
['2.61e-05', '-1.61e-01', '-1.07', '1.15e-05', '1.23', '2.41e-06']
['2.55e-05', '-1.68e-01', '-1.12', '1.11e-05', '1.28', '2.79e-06']
```

### 4.3 Edit **generate_path.py**

Next, modify the contents of **generate_path.py**.Edit the section enclosed by comments from lines 9 to 18.

```python
###### User Input Section Below ##################################################################################

# Required Input
waypoint_file = "angle/angle_ex.txt" # Specify the file that contains waypoints

# Input for generating a path with a new start and goal
s_new = [-0.9539568448800573, 0.14286462209588358, -2.4228928517236046, 8.308474924234588e-05, 2.280063497889632, -0.9539539344058481] # New start joint angles
g_new = [0.8440930198932115, -0.09426596864204129, -2.120484722079457, -5.4759826845440784e-05, 2.2146980161053804, 0.8440667593654201] # New goal joint angles

################################################################################################################
```

For the **waypoint_file**, specify the file path to the text file that was saved in section 4.2.

If generating a path with a new start and goal, provide the new start as a list of joint angles in **s_new**, and provide the new goal as a list of joint angles in **g_new**.

### 4.4 Run **generate_path.py**

Once all the preparations are complete, run **generate_path.py** to generate the new path.

```
python3 generate_path.py
```

Follow the prompts to generate the new path.
