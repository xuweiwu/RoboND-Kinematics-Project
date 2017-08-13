## Project: Kinematics Pick & Place

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/link_assignments.png
[image2]: ./misc_images/ik_position1.png
[image3]: ./misc_images/ik_position2.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points 

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

The link frames are assigned according to the following figure, where joint rotations are about the individual z-axes.

![alt text][image1]

The DH parameters alpha(i-1) can be directly obtained from the above figure:

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | d1 | q1
1->2 | - pi/2 | a1 | 0 | -pi/2 + q2
2->3 | 0 | a2 | 0 | q3
3->4 | - pi/2 | a3 | d4 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->7 (EE) | 0 | 0 | d7 | 0

The other DH parameters (a(i-1), d(i-1)) can then be extracted/calculated from kr210.urdf.xacro file, which is the URDF file (Unified Robot Description Format) for this project. 

From the urdf file one can obtain the relative locations of the joints as shown in the following table:

Joints | Parent Links | Child Links | x | y | z
--- | --- | --- | --- | --- | ---
joint_1 | base_link | link_1 | 0 | 0 | 0.33
joint_2 | link_1 | link_2 | 0.33 | 0 | 0.42
joint_3 | link_2 | link_3 | 0 | 0 | 1.25
joint_4 | link_3 | link_4 | 0.96 | 0 | -0.054
joint_5 | link_4 | link_5 | 0.54 | 0 | 0
joint_6 | link_5 | link_6 | 0.193 | 0 | 0
gripper_joint | link_6 | gripper_link | 0.11 | 0 | 0

Therefore, the remained DH Parameters are:

```python
d1 = 0.33 + 0.42 = 0.75
a1 = 0.35
a2 = 1.25
a3 = -0.054
d4 = 0.96 + 0.54 = 1.50
d7 = 0.193 + 0.11 = 0.303
```

The complete DH parameter table is:

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 | - pi/2 | -0.054 | 1.50 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->7 (EE) | 0 | 0 | 0.303 | 0

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The basic form of the homogeneous transform matrix using the DH parameters can be described as (from frame i-1 to i):

```
T = Matrix([[                cos(qi),                -sin(qi),                0,              a(i-1)],
            [sin(qi)*cos(alpha(i-1)), cos(qi)*cos(alpha(i-1)), -sin(alpha(i-1)), -sin(alpha(i-1))*di],
            [sin(qi)*sin(alpha(i-1)), cos(qi)*sin(alpha(i-1)),  cos(alpha(i-1)),  cos(alpha(i-1))*di],
            [                      0,                       0,                0,                  1]])            
```

Substituting the corresponding DH parameters in the above matrix derives the individual transformation matrices about each joint:

```python
T0_1 = Matrix([[cos(q1), -sin(q1), 0,    0], 
               [sin(q1),  cos(q1), 0,    0], 
               [      0,        0, 1, 0.75], 
               [      0,        0, 0,    1]])   

T1_2 = Matrix([[sin(q2),  cos(q2), 0, 0.35], 
               [      0,        0, 1,    0], 
               [cos(q2), -sin(q2), 0,    0],
               [      0,        0, 0,    1]])

T2_3 = Matrix([[cos(q3), -sin(q3), 0, 1.25], 
               [sin(q3),  cos(q3), 0,    0], 
               [      0,        0, 1,    0], 
               [      0,        0, 0,   1]])

T3_4 = Matrix([[ cos(q4), -sin(q4), 0, -0.054],
               [       0,        0, 1,    1.5], 
               [-sin(q4), -cos(q4), 0,      0], 
               [       0,        0, 0,      1]])

T4_5 = Matrix([[cos(q5), -sin(q5),  0, 0], 
               [      0,        0, -1, 0], 
               [sin(q5),  cos(q5),  0, 0], 
               [      0,        0,  0, 1]])

T5_6 = Matrix([[ cos(q6), -sin(q6), 0, 0], 
               [       0,        0, 1, 0], 
               [-sin(q6), -cos(q6), 0, 0], 
               [       0,        0, 0, 1]])               
               
T6_7 = Matrix([[1, 0, 0,     0], 
               [0, 1, 0,     0], 
               [0, 0, 1, 0.303], 
               [0, 0, 0,     1]])                           
```
Related code can be found in `FK_test.py`, line 35 to 69.

Suppose that the position and orientation of the gripper link are already known, which can be described as the position vector pos_EE

```python
pos_EE = [[px],[py],[pz]]

```

and the rotation matrix R_EE

```
R_EE = RotZ(alpha) * RotY(beta) * RotX(gamma)

```
Herein the convention of x-y-z extrinsic rotations is used, and the alpha, beta, and gamma represent the yaw, pitch, and roll angle respectively.

The homogeneous transform matrices from base_link to gripper_link can then be composed as follows,

```
T0_7 = [[R_EE, pos_EE],
        [0,0,0,1]]

```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and Inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

- Inverse Position Problem
  - Posirion of wrist center wx, wy, wz)
    After evaluating the position vector pos and the rotation matrix R of the end-effctor from the request, the position vector of the wrist center can then be derived as

    ```python
    wx = pos[0] - d7 * R[0,0]
    wy = pos[1] - d7 * R[1,0]
    wz = pos[2] - d7 * R[2,0]

    ```
   	The equation here differs from the lecture notes because the gripper_link is along its x-axis, when no correctional rotation for the difference between conventions of URDF and DH is performed. Related code can be found in `IK_server.py`, line 34 to 36.

  - Calculation of q1, q2, q3
  	From the simplified robot configuration showned below, the value of q1 can be directly caculated by

  	```python
  	q1 = arctan2(wy, wx)

  	```
  	![alt text][image2]

  	

- Inverse Orientation Problem




![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


