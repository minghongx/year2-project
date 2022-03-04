---
title: Week 4 Blog
date: 2022-02-27 15:29:44
tags:
---

**Date:** 21 Feb - 27 Feb

After last week's research, we now have the theoretical knowledge to complete the simulation of three actions. By setting the pitch angle, yaw angle, and roll angle, the angles that motors need to rotate can be calculated. Thus, three actions can be completed. The last task is to complete the writing of the code. In addition, we started the design work for the poster. In the poster, we will explain how the simulation works, and the final result will also be revealed.


## Goals for week 4

1. Finish the code task.
2. Complete the poster design.

## The code of every simulation and demonstration

### Library for the preparation

Before starting to code each action, we first built a library, which is used to prepare for the simulation and set the original parameters.

![library](liba.png)

The code in the figure creates a simulation environment for pybullet and imports the a1 model into it. The initial position and orientation of the model are also set. After that, the IDs of each motor in the model are obtained, and the positions of the motors represented by each ID are listed in a matrix. The matrix has a total of four columns, representing the robot's front right leg, front left leg, hind right leg, and hind left leg. The three rows of the matrix represent three types of motors. The first type is the motor that controls hip abduction and adduction. The second type is the motor that controls hip flexion and extension. The third type is the motor that controls knee joint. This greatly facilitates future work. The robot's leg length, body length, body width, and other information are also set in advance in the code.

![set initial position of motors](liba-set-initial-pos-of-motors.png)

After that, we set the initial positions of the three types of motors respectively.


### Pitch motion

![pitching code](pitching.png)

In the process of writing the code, we found that the model was too light weight. This causes the model to float easily during motion. So we set the gravity to 30 instead of the normal 9.8. Next we set the position of the camera in pybullet to better observe the movement of the robot. As we talked about in our blog last week, to complete the pitch action, we need to set the magnitude of the pitch angle. Through the pitch angle and other set conditions, the position of the foot before and after the movement and the angle that each motor needs to rotate can be calculated. We set up a slide button to smoothly change the magnitude of pitch angle. The pitch range is from -31 rad to +31 rad. Then we use code to complete the calculation of forward kinematics and inverse kinematics to get motor's position. After set motor position, the simulation is completed.

<div align="center">
    <img src="/year2-proj/2022/Week-4-Blog/pitching.gif" width=60% height=60% />
</div>

### Roll motion

![rolling code](rolling.png)

Similar idea to pitching. We set the roll angle range from -0.7 rad to +0.7 rad. We also set a "Reset Position" button to help us reset robot's position easily.

<div align="center">
    <img src="/year2-proj/2022/Week-4-Blog/rolling.gif" width=60% height=60% />
</div>

### Yaw motion

![yawing code](yawing.png)

In the code of yaw motion, we add two extra button which can change the view to top view or front view.


<div align="center">
    <img src="/year2-proj/2022/Week-4-Blog/yawing-initial-view.gif" width=60% height=60% />
</div>

<div align="center">
    <img src="/year2-proj/2022/Week-4-Blog/yawing-front-view.gif" width=60% height=60% />
</div>

<div align="center">
    <img src="/year2-proj/2022/Week-4-Blog/yawing-top-view.gif" width=60% height=60% />
</div>

### Extra motion: squat

![squatting code](squatting.png)

After completing the simulation of the three motions set in the original plan, we designed an extra squat motion. The principle is to complete the action by changing the height of the robot. Different from the previous motion algorithms, in the squat motion we only need to calculate the position of the motor and the foot according to the height of the robot. This squat motion will make the robot jump when the speed of the height change from low to high is too fast. This gave us inspiration, maybe the jumping action of the robot can be realized through this idea.

<div align="center">
    <img src="/year2-proj/2022/Week-4-Blog/squatting.gif" width=60% height=60% />
</div>

## Poster design

![poster design](poster-design.png)

The design of the poster has been completed. The content is currently undergoing further revision and layout. In the poster, we added some bullet points to organise information.

## Week 4 summary

We successfully completed the code work for three actions this week. Even overcompleted the task, designed one more action. The design of the poster has also been completed. The next thing to do is perfect the information in the poster and prepare for our bench inspection next week.
