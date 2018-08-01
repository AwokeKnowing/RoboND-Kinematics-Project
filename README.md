## Project: Kinematics Pick & Place
<img src="pickplace20x.gif" width="960" />
---

[//]: # (Image References)

[image1]: ./layoutwithoffsetsandangles.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README


### Kinematic Analysis

After analyzing the urdf file We come up with the following general layout schematic ( key measured d and a values filled in. see full table below)

![image1]

The full table of DH parameters is as follows:

```
        DH_Table = {
            alpha0:     0, a0:      0, d1: 0.75, q1:        q1,
            alpha1:-pi/2., a1:   0.35, d2:    0, q2:-pi/2.+ q2,
            alpha2:     0, a2:   1.25, d3:    0, q3:        q3,
            alpha3:-pi/2., a3: -0.054, d4:  1.5, q4:        q4,
            alpha4: pi/2., a4:      0, d5:    0, q5:        q5,
            alpha5:-pi/2., a5:      0, d6:    0, q6:        q6,
            alpha6:     0, a6:      0, d7:0.303, q7:         0
        }
```

Each ROW in that table defines the parameters of the transform for a consecutive joint in the arm and gripper.  The links begin from the base, which is taken as the reference frame, and thus has a displacement of 0 and angle of zero.  

To fully define the tranform of from each joint in the arm we have to define all of the transforms consecutively starting from the base. For example, the first transform representing the first joint (T0 to T1) is defined th by the parameters

```
alpha0:     0, a0:      0, d1: 0.75, q1:        q1
```

By applying the following transform matrix to each of those values, we can define the Ti-1 to Ti transform. So the above values plugged in would define the base to first joint (T0 to T1 transform

```
                [     cos(q)       ,      -sin(q)      ,      0     ,      a        ],


                [ sin(q)*cos(alpha), cos(q)*cos(alpha) , -sin(alpha), -sin(alpha)*d ],


                [ sin(q)*sin(alpha), cos(q)*sin(alpha) ,  cos(alpha),  cos(alpha)*d ],


                [        0         ,         0         ,      0     ,      1        ]
 ```



Since all the transforms are specified relative to the previous joint, we can simplify the entire transform from the base to the gripper orientation by multiplying simbolically the consecutive matrices formed by substituting the DH table values into the above matrix row by row.  

```
T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
```

By having this entire homogeneous transfrom from the base to the end effector position, we can then calculate where the gripper will be (relative to the base) for any rotation of of the joints.  So given join rotations and the world position, we know where in the world the gripper is, and in what orientation it is in.  This will be used to solve the Inverse Kinematics problem, where we specify where we would 'like' the gripper to be, and calculate a set of joint rotations that would result in the griper arriving at that location.


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Although there are quite a few joints in the system, solving the inverse kinematics for this particular robot arm structure can be broken down into several smaller tasks that can be computed simply.  Here are the 'short cuts' that let us solve the problem easily.

1. the arm is basically all in one plane, so if we just rotate the base to be in line with where the gripper position should be, then we can solve a 2D problem.
2. If we ignore the gripper and just place the first joints so that they end within 1 gripper-length of the target, we can can simply calculate the angle on a triangle where the base points are defined the the 1st and 3rd joint coordinatesare anywhere 
2. If we consider the seqence of physical joints at the gripper as a 'ball' joint

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


=======
## Project: Kinematics Pick & Place
<img src="pickplace20x.gif" width="960" />
---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Here is an example of how to include an image in your writeup.

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | L1 | qi
1->2 | - pi/2 | L2 | 0 | -pi/2 + q2
2->3 | 0 | 0 | 0 | 0
3->4 |  0 | 0 | 0 | 0
4->5 | 0 | 0 | 0 | 0
5->6 | 0 | 0 | 0 | 0
6->EE | 0 | 0 | 0 | 0


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


