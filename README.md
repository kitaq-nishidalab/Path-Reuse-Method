# Path-Reuse-Method

This document provides an explanation of the code for the new path planning method developed by the Nishida Laboratory.

# 1. Introduction

**Path-Reuse-Method** encodes information from various paths and stores it in the form of "path seeds," which can then be decoded and reused according to the task at hand. Unlike traditional methods that utilize stored primitives, the PR method allows for the expansion and reuse of paths between arbitrary start and goal positions. Additionally, by reusing paths with the desired shape, the number of collision checks in the T-space can be significantly reduced, leading to a substantial decrease in the overall computational cost of path generation.
<br>
<br>

# 2. Operating Environment

- OS: Ubuntu 20.04 LTS
- ROS Noetic
- CPU: Intel Core i7-12700
- Memory: 32 GB
  <br>

# 3. Preparations before using our method

## 3.1 Install ROS Noetic

Please install ROS Noetic on Ubuntu 20.04 LTS.

References: https://wiki.ros.org/noetic/Installation/Ubuntu
<br>
<br>

## 3.2 Setting Up the STOMP Environment

This method utilizes one of the trajectory optimization algorithms known as Stochastic Trajectory Optimization for Motion Planning (STOMP).

You can smoothly set up the environment by using the following repository we have prepared.

https://github.com/kitaq-nishidalab/stomp_ros
<br>
<br>

## 3.3 Setting Up the xArm6 Environment

This planning method is adaptable to the 6-axis robot arm xArm6 provided by uFactory. Therefore, it is necessary to set up the operating environment for xArm6.

You can smoothly set up the environment by using the following repository we have prepared.

https://github.com/kitaq-nishidalab/xarm_ros
<br>
<br>

# 4. Generate new path through encoding and decoding

## 4.1 Download this repository

```
git clone https://github.com/kitaq-nishidalab/Path-Reuse-Method.git
```

Move to the encode&decode folder.

```
cd manual/encode&decode
```

<br>

## 4.2 Save the path information

As a preliminary step, it is necessary to save the path information. The path information consists of a list of joint angles of the robot arm, which needs to be saved in a file.
We save the file containing the path information with the "**.path**" extension.

An example of the text file is shown below:

```
['2.73e-05', '-1.48e-01', '-9.87e-01', '1.25e-05', '1.14', '1.66e-06']
['2.67e-05', '-1.55e-01', '-1.03', '1.20e-05', '1.18', '2.04e-06']
['2.61e-05', '-1.61e-01', '-1.07', '1.15e-05', '1.23', '2.41e-06']
['2.55e-05', '-1.68e-01', '-1.12', '1.11e-05', '1.28', '2.79e-06']
```

<br>

## 4.3 Edit **generate_path.py**

If generating a path with a new start and goal, modify the contents of **generate_path.py**.Provide the new start as a list of joint angles in **s_new**, and provide the new goal as a list of joint angles in **g_new**.

```python
###### User Input Section Below ##################################################################################

# Input for generating a path with a new start and goal
s_new = [-0.9539568448800573, 0.14286462209588358, -2.4228928517236046, 8.308474924234588e-05, 2.280063497889632, -0.9539539344058481] # New start joint angles
g_new = [0.8440930198932115, -0.09426596864204129, -2.120484722079457, -5.4759826845440784e-05, 2.2146980161053804, 0.8440667593654201] # New goal joint angles

################################################################################################################
```

<br>
<br>

## 4.4 Run **generate_path.py**

Once all the preparations are complete, run **generate_path.py** to generate the new path.
For the argument, enter the relative path to the file where the path information saved in section 4.2 is stored.

```bash
python3 generate_path.py angle/angle_ex.path
```

Follow the prompts to generate the new path.
In the prompt, you can choose either to specify a new start and goal to generate a path or to encode the path and save it as a pathseed.
We save the file containing the path seeds with the "**.seed**" extension.
<br>
<br>

# 5. Notice

This program only generates path, so it does not execute it. For instructions on executing the path, please refer to the following README.

# 6. Paper

Thank you for citing Path Reuse Method (SICE FES 2024) if you use any of this code.
```
@inproceedings{pathreuse2024nishida,
  title={Path Encoding and Decoding for Articulated Robots via Information Extraction },
  author={Takeshi Nishida and Tsubasa Watanabe},
  booktitle={Proc. of SICE Festival 2024 with Annual Conf. },
  pages={169-173},
  year={2024},
  organization={SICE}
} 
```
