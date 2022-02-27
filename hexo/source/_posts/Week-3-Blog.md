---
title: Week 3 Blog
date: 2022-02-17 16:09:12
mathjax: true
---
# Week 3 blog
**Date:** 14 Feb - 20 Feb

At the supervisor meeting on Monday, we identified the actions to be done by the simulation. There are three motions in total, pitch motion, yaw motion and roll motion. The reason for choosing these three actions is that we have learned the correct way to control the robot to complete the action by consulting the data. Instead of the combination of simple movements of multiple joints that we thought before to complete the action. In fact, the correct theory should be to control the motor by calculating the coordinates of each key point on the robot body before and after the movement. The three actions we chose apply the idea of the simulation in three directions of space, respectively. This method requires modeling the robot and applying transformation matrices to calculate the hip and foot positions.


## Goals for week 3

1. Research investigates kinematic modeling of quadruped robots.
2. Study the matrix transformation of the coordinate system to calculate the coordinates before and after the action.

## Motion computation theory

### The modeling calculation of pitch motion

![leg modeling diagram](Leg-modeling-diagram.jpg)

The legs of a quadruped robot can be simplified as a two-bar linkage. In the figure below, $\theta\_1$ is the hip joint angle, $\theta\_2$ is the elbow joint angle, L$\_{1}$ is the length of the thigh, L$\_{2}$ is the length of the calf, point P represents the foot, and the x-axis represents the robot body. Among them, L$\_{1}$ and L$\_{2}$ are known, and the initial height of the robot is also fixed, so $\theta\_1$ and $\theta\_2$ must be two determined values, which can be obtained by calculation. From this, we can determine the coordinates of point P(x,y).

![pitch motion model](pitch-motion-model.jpg)

Next, we draw the modeling diagram of the pitch motion. In the picture above, point O is the center point of the robot's torso, point A is the coordinates of the hip joint before movement, point B is the coordinates after the movement of the hip joint, L is the distance from the hip joint to the midpoint of the body in the current plane and $\alpha$ is the angle of leaning over. In the above steps we got the coordinates of point P in the stationary state. Next, we move the coordinate system from point A to point O, then rotate the coordinate system clockwise by $\alpha$ angle, and then translate to point B. Find the coordinates of point P in the current coordinate system of point B, and the work is completed. This uses the coordinate system transformation matrix and coordinate transformation matrix knowledge in linear algebra.

A$\rightarrow$O coordinate system transformation matrix:

$$
T_1=
  \begin{bmatrix}
    1 & 0 & L \\\\
    0 & 1 & 0 \\\\
    0 & 0 & 1 \\\\
    \end{bmatrix}
$$

O$\rightarrow$B coordinate system transformation matrix:

$$
T_2=
  \begin{bmatrix}
    cos\alpha & -sin\alpha & -L \\\\
    sin\alpha & cos\alpha & 0 \\\\
    0 & 0 & 1 \\\\
    \end{bmatrix}
$$

The coordinate of point P in the coordinate system of point B is:

$$ P_B=T_1T_2P_A $$

Next, the angles $\theta\_1$ and $\theta\_2$ at the two joints can be solved by inverse kinematics. Calculate as follows:

$$ cos\theta_2=\frac{-L^2_1-L^2_2+x^2+y^2}{2L_1L_2} $$
$$ sin\theta_2=\sqrt{\smash[b]{1-cos^2\alpha}} $$
$$ \theta_2=atan2(sin\theta_2,cos\theta_2) $$
$$ \theta_1=atan2(y,x)-atan2(L_2sin\theta_2,L_1+L_2cos\theta_2) $$

With the changed angles $\theta\_1$ and $\theta\_2$ , we know the angle that the motors at the two joints need to rotate.

In summary, once we have designed the $\alpha$ angle, the simulation of the pitch motion can be completed.

### The modeling calculation of roll motion

![roll motion model](roll-motion-model.jpg)

W is the distance from the hip joint to the midpoint of the fuselage in the current plane. $\beta$ is the angle of rolling. Others are similar to the pitch motion.

The two transformation matrices are:

$$ T_1=
  \begin{bmatrix}
    1 & 0 & 0 & 0 \\\\
    0 & 1 & 0 & W \\\\
    0 & 0 & 1 & 0 \\\\
    0 & 0 & 0 & 1 \\\\
    \end{bmatrix}
$$
$$ T_2=
  \begin{bmatrix}
    1 & 0 & 0 & 0 \\\\
    0 & cos\beta & -sin\beta & -W \\\\
    0 & sin\beta & cos\beta & 0 \\\\
    0 & 0 & 0 & 1 \\\\
    \end{bmatrix}
$$

The coordinates of point P after the movement are:

$$ P'=T_2T_1P $$

Then, the two angles at the joint can be solved by inverse kinematics.

### The modeling calculation of yaw motion

![yaw motion model](yaw-motion-model.jpg)

$\gamma$ is the angle of yawing.

The two transformation matrices are:

$$ T_1=
  \begin{bmatrix}
    1 & 0 & 0 & L \\\\
    0 & 1 & 0 & W \\\\
    0 & 0 & 1 & 0 \\\\
    0 & 0 & 0 & 1 \\\\
    \end{bmatrix}
$$
$$ T_2=
  \begin{bmatrix}
    cos\gamma & -sin\gamma & 0 & -L \\\\
    sin\gamma & cos\gamma & 0 & -W \\\\
    0 & 0 & 1 & 0 \\\\
    0 & 0 & 0 & 1 \\\\
    \end{bmatrix}
$$

The coordinates of point P after the movement are:

$$ P'=T_2T_1P $$

Then, the two angles at the joint can be solved by inverse kinematics.

## Week 3 summary

This week we successfully modeled three actions. After modeling, the changes of each variable before and after the action were deduced. The feasibility of the project is verified at the theoretical level. Next, we will start writing the code to complete the simulation of the three actions in pybullet.

