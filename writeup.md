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

[image1]: ./misc_images/link_assignments.jpg
[image2]: ./misc_images/ik_position1.jpg
[image3]: ./misc_images/ik_position2.jpg
[image4]: ./misc_images/result.jpg

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

The other constant DH parameters ( a(i-1), d(i-1) ) can then be extracted/calculated from kr210.urdf.xacro file, which is the URDF (Unified Robot Description Format) file for this project. 

From the URDF file one can obtain the relative locations of the joints as shown in the following table:

Joints | Parent Links | Child Links | x | y | z
--- | --- | --- | --- | --- | ---
joint_1 | base_link | link_1 | 0 | 0 | 0.33
joint_2 | link_1 | link_2 | 0.33 | 0 | 0.42
joint_3 | link_2 | link_3 | 0 | 0 | 1.25
joint_4 | link_3 | link_4 | 0.96 | 0 | -0.054
joint_5 | link_4 | link_5 | 0.54 | 0 | 0
joint_6 | link_5 | link_6 | 0.193 | 0 | 0
gripper_joint | link_6 | gripper_link | 0.11 | 0 | 0

Therefore, the remained constant DH Parameters are:
```
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

Using the DH parameters, the homogeneous transform matrix from frame i-1 to i can be described as:

```
T = [[                cos(qi),                -sin(qi),                0,              a(i-1)],
     [sin(qi)*cos(alpha(i-1)), cos(qi)*cos(alpha(i-1)), -sin(alpha(i-1)), -sin(alpha(i-1))*di],
     [sin(qi)*sin(alpha(i-1)), cos(qi)*sin(alpha(i-1)),  cos(alpha(i-1)),  cos(alpha(i-1))*di],
     [                      0,                       0,                0,                  1]]      
```

Substituting the corresponding DH parameters in the above matrix derives the individual transformation matrices about each joint:

```
T0_1 = [[cos(q1), -sin(q1), 0,    0], 
        [sin(q1),  cos(q1), 0,    0], 
        [      0,        0, 1, 0.75], 
        [      0,        0, 0,    1]]

T1_2 = [[sin(q2),  cos(q2), 0, 0.35], 
        [      0,        0, 1,    0], 
        [cos(q2), -sin(q2), 0,    0],
        [      0,        0, 0,    1]]

T2_3 = [[cos(q3), -sin(q3), 0, 1.25], 
        [sin(q3),  cos(q3), 0,    0], 
        [      0,        0, 1,    0], 
        [      0,        0, 0,   1]]

T3_4 = [[ cos(q4), -sin(q4), 0, -0.054],
        [       0,        0, 1,    1.5], 
        [-sin(q4), -cos(q4), 0,      0], 
        [       0,        0, 0,      1]]

T4_5 = [[cos(q5), -sin(q5),  0, 0], 
        [      0,        0, -1, 0], 
        [sin(q5),  cos(q5),  0, 0], 
        [      0,        0,  0, 1]]

T5_6 = [[ cos(q6), -sin(q6), 0, 0], 
        [       0,        0, 1, 0], 
        [-sin(q6), -cos(q6), 0, 0], 
        [       0,        0, 0, 1]]          
               
T6_7 = [[1, 0, 0,     0], 
        [0, 1, 0,     0], 
        [0, 0, 1, 0.303], 
        [0, 0, 0,     1]]                         
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

- Inverse Position Problem (related code can be found in `IK_server.py`, line 32 to 76)
  - Calculate the position of wrist center (wx, wy, wz)
  
    After evaluating the position vector pos_EE and the rotation matrix R_EE of the end-effctor, the position vector of the wrist center can be obtained by
    ```
    wx = pos_EE[0] - d7 * R_EE[0,0]
    wy = pos_EE[1] - d7 * R_EE[1,0]
    wz = pos_EE[2] - d7 * R_EE[2,0]
    ```
    The above equation differs from the lecture notes because the gripper_link is along its x-axis, when no correctional rotation is performed for the difference between conventions of URDF and DH.

  - Calculate q1, q2, q3
  
    From the simplified robot configuration shown below, the value of q1 can be directly caculated by
    ```
    q1 = arctan2(wy, wx)
    ```
    ![alt text][image2]
    
    The values of q2 and q3 can be obtained by using the laws of cosine according to the following diagram.
    ![alt text][image3]
    
    Intermediate parameters needed for the calculations are
    ```
    # Distance between O2 and WC projected on x1-axis
    wx1 = sqrt(wx**2 + wy**2) - a1
    # Distance between O2 and WC projected on z1-axis
    wz1 = wz - d1
    # Distance between O2 and O3
    l2 = a2
    # Distance between O3 and WC
    l3 = sqrt(a3**2 + d4**2)
    ```
    
    For q3:
    ```
    # The cosine law is applied to calculate cos(pi/2 - q3) = sin(q3)
    sq3 = (l2**2 + l3**2 - (wx1**2 + wz1**2))/(2*l2*l3)
    # A positive cq3 implies that q3 is between -pi/2 and pi/2
    # It is a desired range for q3 because uncontinuity will not occur
    cq3 = sqrt(1 - sq3**2)
    
    # Acoording to the URDF file the limit of joint 3 is [-210, 65]deg    
    # If the resulted q3 is larger than 65 deg, the sign of cq3 needs to be changed
    atanq3 = arctan2(sq3, cq3)
    if atanq3 > 65/180*pi and sq3 < 0:    	
    	q3 = atanq3 - arctan2(-a3, d4) - pi
    else:
    	q3 = atanq3 - arctan2(-a3, d4)
    ```
    
    For q2:
    ```
    # The cosine law is applied to calculate cos(phi)
    cphi = (wx1**2 + wz1**2 + l2**2 - l3**2)/(2*l2*sqrt(wx1**2 + wz1**2))
    # A positive sphi implies that phi is between 0 to pi 
    sphi = sqrt(1 - cphi**2)
    
    # Acoording to the URDF file the limit of joint 2 is [-45, 85]deg
    # The sign of phi should be the same as the sign of arctan2(wx1, wz1)
    # If q3 < -pi/2, then arctan2(wx1, wz1) < 0, the sign of cq3 needs to be changed
    if q3 > -pi/2:
    	q2 = arctan2(wx1, wz1) - arctan2(sphi, cphi)
    else:
    	q2 = arctan2(wx1, wz1) - arctan2(-sphi, cphi)
    ```
    
- Inverse Orientation Problem (related code can be found in `IK_server.py`, line 78 to 99)
  - Calculate R3_6
  
    The equation to calculate R3_6 is:
    ```
    R3_6= inv(R0_3) * R_EE * inv(R_gripper_corr)
    ```
    where the symbolic expressions of R0_3 and R_gripper_corr can be evaluated and simplified in advance to reduce the computation load.
    
   - Calculate q4, q5, q6
   
     R3_6 can be expressed with q4,q5, and q6 as follows
     ```
     R3_6 = [[-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4)],
             [                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5)],
             [-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5)]]
     ```
     
     Using atan2 function, the values of q4, q5, and q6 can be obtained by
     ```
     q4 = arctan2(R3_6[2,2], -R3_6[0,2])
     q5 = arctan2(sq5, R3_6[1,2])
     q6 = arctan2(-R3_6[1,1], R3_6[1,0])
     ```
     To get through the wrist singularity, q4 will be set to zero if q5 is below a predefined limit. This part of code is learned from the function euler_from_matrix in transformation.py, line 1062 to 1080.
     ```     
     eps = finfo(float).eps * 4.0
     sq5 = sqrt(R3_6[1,0]**2 + R3_6[1,1]**2)
     if sq5 > eps:
         q4 = arctan2(R3_6[2,2], -R3_6[0,2])
         q5 = arctan2(sq5, R3_6[1,2])
         q6 = arctan2(-R3_6[1,1], R3_6[1,0])
     else:
         q4 = arctan2(-R3_6[0,1], R3_6[0,0])
         q5 = 0
         q6 = 0
     ```
     
### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

The original In oder to improve the performance, the symbolic functions are replaced with numpy functions within `IK_server.py`, since only two symbolic expressions (R0_3 and R_gripper_corr) are actually needed during the online computations. The implemented code can lead to a completion of 10/10 pick and place cycles, as shown below

![alt text][image4]
