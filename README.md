## Project: Kinematics Pick & Place
<img src="pickplace20x.gif" width="960" />
---

[//]: # (Image References)

[image1]: ./layoutwithoffsetsandangles.png
[image2]: ./baserotation.png
[image3]: ./elbowangle.png
[image4]: ./ballwrist.png


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


#### Inverse Kinematics

Although there are quite a few joints in the system, solving the inverse kinematics for this particular robot arm structure can be broken down into several smaller tasks that can be computed simply.  Here are the 'short cuts' that let us solve the problem easily.

1. the arm is basically all in one plane, so if we just rotate the base to be in line with where the gripper position should be, then we can solve a 2D problem.

![image2]

We know the length of all sides of the right triangle and extract the 2 angles, and use the one at the base

2. If we ignore the gripper and just place the first joints so that they end within 1 gripper-length of the target, we can can simply calculate the angle on a triangle where the base points are defined the the 1st and 3rd joint coordinates 

![image3]

So here again we know the length of the joints and calculate the length of the red base as the hypotenuse of the right (blue) triangle. We calculate the two base angles with sin and cos and then the final actual joint angle as the arctangent using the atan2 function which will work with proper signs according to the quadrant we're in.

3. If we consider the seqence of 2 physical joints at the gripper as a 'ball' joint by 'logically sliding' the center of rotation of the second joint to be over the first, then we can rotate the calculate the rotation of the gripper joints as just the orientation of the target pose.

![image4]

So we simple do the same process of finding the angle based on a triangle where the angle we want represents the difference in rotation between our current gripper orientation and the target gripper orientation.


### Project Implementation
The actual python implemntation is contained in the IK_server.py file.  The project is set up where Gazebo loads the arm model, and rviz also loads of the model, and moveit! is used to supply a list of around 20-50 intermediate points on a path between where the gripper is now, and where the next desired position is.  The K_server.py file then calculates the joint rotations needed to get to each consecutive end effector position-orientation in the path using the method discussed above.

### Results

Using the above method, and the supplied pre-configured environment, we are able to consistently retrieve the objects.  Ocasionally the path planner adds points on a path which extend a joint to it's limits, causing undefined values, due to the 'triangles' getting squashed into 0 height.  We caught one such attempt in the below animation.  The arm is expected to follow a very wild, high path, and ends up with a zero value, causing it to end up dropping the item and starting again.  In the future this could be mitigated by either placing constraints on the path, or by placing constraints on the elbow joint angles to ensure they never rotate to a completely flat position.  Nevertheless, the average success rate was 9.5/10, and could be brought near 10/10 by saving paths that do not unnecessarily extend the joints.

<img src="pickplace20x.gif" width="960" />





