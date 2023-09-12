import sys
import numpy as np
from copy import deepcopy
from math import pi

import rospy
# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector

# for timing that is consistent with simulation or real time as appropriate
from core.utils import time_in_seconds

# forward and inverse kinematics
from calculateIK6 import IK
from calculateFK import FK

class Final:

    def rotation(self, trans_matrix):
        T = trans_matrix['T']
        print("TTTTTT", T)
        # Determine perpendicular component
        x_vec = abs(T[2,0])
        y_vec = abs(T[2,1])
        z_vec = abs(T[2,2])

        # Initialize flag values
        x_flag = 0
        y_flag = 0
        z_flag = 0

        # Determine vector perpendicular to block
        if x_vec > y_vec and x_vec > z_vec:
            x_flag = 1
        elif y_vec > x_vec and y_vec > z_vec:
            y_flag = 1
        elif z_vec > x_vec and z_vec > y_vec:
            z_flag = 1

        if T[2,0] > 0 and x_flag == 1: # x axis up

            # Define empirically derived transformation matrix
            T_trans = np.array([[0, 0, -1, 0], \
                [0, 1, 0, 0], \
                [1, 0, 0, 0], \
                [0, 0, 0, 1]])

        elif T[2,0] < 0 and x_flag == 1: # x axis down

            # Define empirically derived transformation matrix
            T_trans = np.array([[0, 0, 1, 0], \
                [0, 1, 0, 0], \
                [-1, 0, 0, 0], \
                [0, 0, 0, 1]])

        elif T[2,1] > 0 and y_flag == 1: # y axis up

            # Define empirically derived transformation matrix
            T_trans = np.array([[1, 0, 0, 0], \
                [0, 0, -1, 0], \
                [0, 1, 0, 0], \
                [0, 0, 0, 1]])

        elif T[2,1] < 0 and y_flag == 1: # y axis down

            # Define empirically derived transformation matrix
            T_trans = np.array([[1, 0, 0, 0], \
                [0, 0, 1, 0], \
                [0, -1, 0, 0], \
                [0, 0, 0, 1]])

        elif T[2,2] > 0 and z_flag == 1: # z axis up

            # Define empirically derived transformation matrix
            T_trans = np.array([[1, 0, 0, 0], \
                [0, -1, 0, 0], \
                [0, 0, -1, 0], \
                [0, 0, 0, 1]])

        elif T[2,2] < 0 and z_flag == 1: # z axis down

            # Define empirically derived transformation matrix
            T_trans = np.array([[1, 0, 0, 0], \
                [0, 1, 0, 0], \
                [0, 0, 1, 0], \
                [0, 0, 0, 1]])

        return T_trans

    def updated_cube_finder(self, omega, time_delay, cube_matrix):

        # omega is speed of table in angular velocity (deg/s)
        # time_delay is measured time between detection position and grasping of cube
        # cube_matrix is matrix of dynamic block as measured from get_detection
        print("Block Matrix at Detection: \n", cube_matrix)

        # Determine absolute angular change in radians
        omega_rad = omega*(pi/180)
        rad_change = omega_rad * time_delay
        print("Angular Change of ", rad_change)

        # Use rotation from Final to determine "corrected" dynamic block matrix
        cube_matrix_dict = {'T': cube_matrix}
        cube_matrix_rotation = self.rotation(cube_matrix_dict)
        cube_matrix_corrected = cube_matrix @ cube_matrix_rotation
        print("Corrected Cube Matrix: \n", cube_matrix_corrected)

        # Determine approximately where on the table the block is located (dist from center)
        x_comp = cube_matrix_corrected[0, 3]
        y_comp = 0.9 - np.abs(cube_matrix_corrected[1, 3]) # hard code for blue
        radial_dist = np.sqrt(x_comp**2 + y_comp**2)
        print("Calculated Radius of Block from Center of Turntable", radial_dist)
        print("Detected x Position from Center of Turntable", x_comp)
        print("Detected y Position from Center of Turntable", y_comp)

        # Determine angle swept out by block at detection point
        theta_init = np.arccos(x_comp / radial_dist)

        # Determine new x and y position
        new_x = radial_dist * np.cos(theta_init + rad_change)
        new_y = radial_dist * np.sin(theta_init + rad_change)
        new_z = cube_matrix[2, 3]
        print("Predicted x Position from Center of Turntable", new_x)
        print("Predicted y Position from Center of Turntable", new_y)

        # Determine change in orientations of new_x and new_y vectors
        cube_matrix_orientation = cube_matrix_corrected[0:3, 0:3]
        rotation_matrix = np.array([[np.cos((2*pi)-rad_change), -np.sin((2*pi)-rad_change), 0], \
            [np.sin((2*pi)-rad_change), np.cos((2*pi)-rad_change), 0], \
            [0, 0, 1]]) # recall that we must subtract from 2*pi because axis is in negative z-dir
        print("Rotation Matrix from Angular Movement \n", rotation_matrix)
        updated_orientation = cube_matrix_orientation @ rotation_matrix
        print("New Orientation after Rotation: \n", updated_orientation)

        # Create and populate new output matrix for grasping
        updated_cube_matrix = np.zeros([4, 4])
        updated_cube_matrix[0:3, 0:3] = updated_orientation[0:3, 0:3]
        updated_cube_matrix[3, 3] = 1
        updated_cube_matrix[0, 3] = new_x
        updated_cube_matrix[1, 3] = -0.9 + new_y
        updated_cube_matrix[2, 3] = new_z
        print("Updated Cube Matrix for Grasping: \n", updated_cube_matrix)

        return updated_cube_matrix


if __name__ == "__main__":
    try:
        team = rospy.get_param("team") # 'red' or 'blue'
    except KeyError:
        print('Team must be red or blue - make sure you are running final.launch!')
        exit()

    rospy.init_node("team_script")
    arm = ArmController()
    detector = ObjectDetector()

    start_position = np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866])
    arm.safe_move_to_position(start_position) # on your mark!

    print("\n****************")
    if team == 'blue':
        team_multi = -1
        print("** BLUE TEAM  **")
    else:
        team_multi = 1
        print("**  RED TEAM  **")
    print("****************")
    input("\nWaiting for start... Press ENTER to begin!\n") # get set!
    print("Go!\n") # go!

    # STUDENT CODE HERE
    fk = FK()
    ik = IK()
    final = Final()
    COUNT_GRIP = 0  # count how many blocks the grippers successfully grasped and stacked
    ########################################
    # Detection Position for Static Blocks #
    ########################################
    detect_matrix = np.array([[1,0,0,0.5],[0,-1,0,(-0.12)*team_multi],[0,0,-1,0.6],[0,0,0,1]])
    detect_target = {'R': detect_matrix[0:3, 0:3], 't': detect_matrix[0:3, 3]}
    detect_pos = ik.panda_ik(detect_target)[0]

    print(detect_pos)
    arm.safe_move_to_position(detect_pos)
    # get the transform from camera to panda_end_effector
    H_ee_camera = detector.get_H_ee_camera()
    print(H_ee_camera)
    # Detect some blocks...
    cube1_static = np.eye(4)
    cube2_static = np.eye(4)
    cube3_static = np.eye(4)
    cube4_static = np.eye(4)

    for (name, pose) in detector.get_detections():
        if 'static' in name:
            cube1_static = pose
            cube1_name = name

        print(name,'\n',pose)

    for (name, pose) in detector.get_detections():
        if 'static' in name and name != cube1_name:
            cube2_static = pose
            cube2_name = name

    for (name, pose) in detector.get_detections():
        if 'static' in name and name != cube1_name and name != cube2_name:
            cube3_static = pose
            cube3_name = name

    for (name, pose) in detector.get_detections():
        if 'static' in name and name != cube1_name and name != cube2_name and name != cube3_name:
            cube4_static = pose
            cube4_name = name

    # Get the position and orientation of blocks in world frame


    T0_e = fk.forward(detect_pos)[1]
    print("Matrix of Dectection: \n", T0_e)

    T0_cube1 = T0_e @ H_ee_camera @ cube1_static
    T0_cube2 = T0_e @ H_ee_camera @ cube2_static
    T0_cube3 = T0_e @ H_ee_camera @ cube3_static
    T0_cube4 = T0_e @ H_ee_camera @ cube4_static

    print("T cube 1 before rotation: \n", T0_cube1)
    print("T cube 2 before rotation: \n", T0_cube2)
    print("T cube 3 before rotation: \n", T0_cube3)
    print("T cube 4 before rotation: \n", T0_cube4)

    trans_matrix1 = {'T': T0_cube1}
    trans_matrix2 = {'T': T0_cube2}
    trans_matrix3 = {'T': T0_cube3}
    trans_matrix4 = {'T': T0_cube4}
    cube1 = T0_cube1 @ final.rotation(trans_matrix1)
    cube2 = T0_cube2 @ final.rotation(trans_matrix2)
    cube3 = T0_cube3 @ final.rotation(trans_matrix3)
    cube4 = T0_cube4 @ final.rotation(trans_matrix4)

    cube1_inter = cube1
    cube2_inter = cube2
    cube3_inter = cube3
    cube4_inter = cube4

    cube1_inter[2,3] = cube1_inter[2,3] + 0.1
    cube2_inter[2,3] = cube2_inter[2,3] + 0.1
    cube3_inter[2,3] = cube3_inter[2,3] + 0.1
    cube4_inter[2,3] = cube4_inter[2,3] + 0.1

    cube1_IK = T0_cube1 @ final.rotation(trans_matrix1)
    cube2_IK = T0_cube2 @ final.rotation(trans_matrix2)
    cube3_IK = T0_cube3 @ final.rotation(trans_matrix3)
    cube4_IK = T0_cube4 @ final.rotation(trans_matrix4)

    cube1_IK[2,3] = cube1_IK[2,3] - 0.015
    cube2_IK[2,3] = cube2_IK[2,3] - 0.015
    cube3_IK[2,3] = cube3_IK[2,3] - 0.015
    cube4_IK[2,3] = cube4_IK[2,3] - 0.015

    print("Matrix of Cube 1: \n", cube1_IK)
    print("Matrix of Cube 2: \n", cube2_IK)
    print("Matrix of Cube 3: \n", cube3_IK)
    print("Matrix of Cube 4: \n", cube4_IK)
    print("cube1_inter", cube1_inter)
    print("cube2_inter", cube2_inter)
    print("cube3_inter", cube3_inter)
    print("cube4_inter", cube4_inter)


    arm.exec_gripper_cmd(0.12, 50)

    IK_input = np.array([[0, -1, 0, 0],[1, 0, 0, 0],[0, 0, -1, 0],[0, 0, 0, 1]])

####### Cube 1 ########
    # Move around...

    # Rotation Matrix Troubleshooting
    rot = np.array([[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    print("IK for Cube 1 Intermediate: \n", cube1_inter)
    cube1_inter_pos = []
    count_rot = 0
    while len(cube1_inter_pos) == 0 and count_rot < 4:
        cube1_inter_target = {'R': cube1_inter[0:3, 0:3], 't': cube1_inter[0:3, 3]}
        cube1_inter_pos = ik.panda_ik(cube1_inter_target)

        if len(cube1_inter_pos) == 0:
            cube1_inter = cube1_inter @ rot
            count_rot = count_rot + 1

    print("IK for Cube 2 Intermediate: \n", cube2_inter)
    cube2_inter_pos = []
    count_rot = 0
    while len(cube2_inter_pos) == 0 and count_rot < 4:
        cube2_inter_target = {'R': cube2_inter[0:3, 0:3], 't': cube2_inter[0:3, 3]}
        cube2_inter_pos = ik.panda_ik(cube2_inter_target)

        if len(cube2_inter_pos) == 0:
            cube2_inter = cube2_inter @ rot
            count_rot = count_rot + 1

    print("IK for Cube 3 Intermediate: \n", cube3_inter)
    cube3_inter_pos = []
    count_rot = 0
    while len(cube3_inter_pos) == 0 and count_rot < 4:
        cube3_inter_target = {'R': cube3_inter[0:3, 0:3], 't': cube3_inter[0:3, 3]}
        cube3_inter_pos = ik.panda_ik(cube3_inter_target)

        if len(cube3_inter_pos) == 0:
            cube3_inter = cube3_inter @ rot
            count_rot = count_rot + 1

    print("IK for Cube 4 Intermediate: \n", cube4_inter)
    cube4_inter_pos = []
    count_rot = 0
    while len(cube4_inter_pos) == 0 and count_rot < 4:
        cube4_inter_target = {'R': cube4_inter[0:3, 0:3], 't': cube4_inter[0:3, 3]}
        cube4_inter_pos = ik.panda_ik(cube4_inter_target)

        if len(cube4_inter_pos) == 0:
            cube4_inter = cube4_inter @ rot
            count_rot = count_rot + 1

    print("IK for Cube 1 Final: \n", cube1_IK)
    cube1_pos = []
    count_rot = 0
    while len(cube1_pos) == 0 and count_rot < 4:
        cube1_target = {'R': cube1_IK[0:3, 0:3], 't': cube1_IK[0:3, 3]}
        cube1_pos = ik.panda_ik(cube1_target)

        if len(cube1_pos) == 0:
            cube1_IK = cube1_IK @ rot
            count_rot = count_rot + 1

    print("IK for Cube 2 Final: \n", cube2_IK)
    cube2_pos = []
    count_rot = 0
    while len(cube2_pos) == 0 and count_rot < 4:
        cube2_target = {'R': cube2_IK[0:3, 0:3], 't': cube2_IK[0:3, 3]}
        cube2_pos = ik.panda_ik(cube2_target)

        if len(cube2_pos) == 0:
            cube2_IK = cube2_IK @ rot
            count_rot = count_rot + 1

    print("IK for Cube 3 Final: \n", cube3_IK)
    cube3_pos = []
    count_rot = 0
    while len(cube3_pos) == 0 and count_rot < 4:
        cube3_target = {'R': cube3_IK[0:3, 0:3], 't': cube3_IK[0:3, 3]}
        cube3_pos = ik.panda_ik(cube3_target)

        if len(cube3_pos) == 0:
            cube3_IK = cube3_IK @ rot
            count_rot = count_rot + 1

    print("IK for Cube 4 Final: \n", cube4_IK)
    cube4_pos = []
    count_rot = 0
    while len(cube4_pos) == 0 and count_rot < 4:
        cube4_target = {'R': cube4_IK[0:3, 0:3], 't': cube4_IK[0:3, 3]}
        cube4_pos = ik.panda_ik(cube4_target)

        if len(cube4_pos) == 0:
            cube4_IK = cube4_IK @ rot
            count_rot = count_rot + 1

    cube1_inter_pos = cube1_inter_pos[0]
    cube2_inter_pos = cube2_inter_pos[0]
    cube3_inter_pos = cube3_inter_pos[0]
    cube4_inter_pos = cube4_inter_pos[0]
    cube1_pos = cube1_pos[0]
    cube2_pos = cube2_pos[0]
    cube3_pos = cube3_pos[0]
    cube4_pos = cube4_pos[0]

################################
######## Static Cube 1 #########
################################
    print("Going to Cube 1 Intermediate: \n", cube1_inter_pos)
    arm.safe_move_to_position(cube1_inter_pos)
    print("Going to Cube 1 Final: \n", cube1_pos)
    arm.safe_move_to_position(cube1_pos)
    arm.exec_gripper_cmd(0.03, 50)
    gripper_state = arm.get_gripper_state()
    print("Current State: \n", arm.get_gripper_state())
    gripper_pos = gripper_state['position']
    if (gripper_pos[0] + gripper_pos[1]) < 0.055 and (gripper_pos[0] + gripper_pos[1]) > 0.02:
        COUNT_GRIP = COUNT_GRIP + 1
    print(COUNT_GRIP, " blocks are grasped")
    arm.safe_move_to_position(detect_pos)

    cube1_stack= np.array([[1, 0, 0, 0.562],[0, -1, 0, team_multi*0.2],[0, 0, -1, 0.03+0.05*(COUNT_GRIP - 1)+0.225380201],[0, 0, 0, 1]])
    stack1_target = {'R': cube1_stack[0:3, 0:3], 't': cube1_stack[0:3, 3]}
    stack1_pos = ik.panda_ik(stack1_target)[0]

    print("Stacking Cube 1: \n", stack1_pos)
    arm.safe_move_to_position(stack1_pos)
    arm.exec_gripper_cmd(0.12, 50)
    print("Arm State: \n", arm.get_gripper_state())

    cube1_inter_stack = cube1_stack
    cube1_inter_stack[2,3] = cube1_inter_stack[2,3] + 0.1
    stack1_inter_target = {'R': cube1_inter_stack[0:3, 0:3], 't': cube1_inter_stack[0:3, 3]}
    stack1_inter_pos = ik.panda_ik(stack1_inter_target)[0]
    arm.safe_move_to_position(stack1_inter_pos)

    arm.safe_move_to_position(detect_pos)
#########################
######## Cube 2 #########
#########################
    print("Going to Cube 2 Intermediae: \n", cube2_inter_pos)
    arm.safe_move_to_position(cube2_inter_pos)
    print("Going to Cube 2 Final: \n", cube2_pos)
    arm.safe_move_to_position(cube2_pos)
    arm.exec_gripper_cmd(0.03, 50) # Close grippers to grasp the block
    gripper_state = arm.get_gripper_state()
    print("Current State: \n", arm.get_gripper_state())
    gripper_pos = gripper_state['position']
    if (gripper_pos[0] + gripper_pos[1]) < 0.055 and (gripper_pos[0] + gripper_pos[1]) > 0.02:
        COUNT_GRIP = COUNT_GRIP + 1
    print(COUNT_GRIP, " blocks are grasped")
    arm.safe_move_to_position(detect_pos)

    cube2_stack= np.array([[1, 0, 0, 0.562],[0, -1, 0, team_multi*0.2],[0, 0, -1, 0.03+0.05*(COUNT_GRIP - 1)+0.225380201],[0, 0, 0, 1]])
    stack2_target = {'R': cube2_stack[0:3, 0:3], 't': cube2_stack[0:3, 3]}
    stack2_pos = ik.panda_ik(stack2_target)[0]

    print("Stacking Cube 2: \n", stack2_pos)
    arm.safe_move_to_position(stack2_pos)
    arm.exec_gripper_cmd(0.12, 50)
    print("Arm State: \n", arm.get_gripper_state())

    cube2_inter_stack = cube2_stack
    cube2_inter_stack[2,3] = cube2_inter_stack[2,3] + 0.1
    stack2_inter_target = {'R': cube2_inter_stack[0:3, 0:3], 't': cube2_inter_stack[0:3, 3]}
    stack2_inter_pos = ik.panda_ik(stack2_inter_target)[0]

    arm.safe_move_to_position(stack2_inter_pos)
    arm.safe_move_to_position(detect_pos)
#########################
######## Cube 3 #########
#########################
    print("Going to Cube 3 Intermediate: \n", cube3_inter_pos)
    arm.safe_move_to_position(cube3_inter_pos)
    print("Going to Cube 3 Final: \n", cube3_pos)
    arm.safe_move_to_position(cube3_pos)
    arm.exec_gripper_cmd(0.03, 50)
    gripper_state = arm.get_gripper_state()
    print("Current State: \n", arm.get_gripper_state())
    gripper_pos = gripper_state['position']
    if (gripper_pos[0] + gripper_pos[1]) < 0.055 and (gripper_pos[0] + gripper_pos[1]) > 0.02:
        COUNT_GRIP = COUNT_GRIP + 1
    print(COUNT_GRIP, " blocks are grasped")
    arm.safe_move_to_position(detect_pos)

    cube3_stack= np.array([[1, 0, 0, 0.562],[0, -1, 0, team_multi*0.2],[0, 0, -1, 0.03+0.05*(COUNT_GRIP - 1)+0.225380201],[0, 0, 0, 1]])
    stack3_target = {'R': cube3_stack[0:3, 0:3], 't': cube3_stack[0:3, 3]}
    stack3_pos = ik.panda_ik(stack3_target)[0]

    print("Stacking Cube 3: \n", stack3_pos)
    arm.safe_move_to_position(stack3_pos)
    arm.exec_gripper_cmd(0.12, 50)
    print("Arm State: \n", arm.get_gripper_state())

    cube3_inter_stack = cube3_stack
    cube3_inter_stack[2,3] = cube3_inter_stack[2,3] + 0.1
    stack3_inter_target = {'R': cube3_inter_stack[0:3, 0:3], 't': cube3_inter_stack[0:3, 3]}
    stack3_inter_pos = ik.panda_ik(stack3_inter_target)[0]
    arm.safe_move_to_position(stack3_inter_pos)

    arm.safe_move_to_position(detect_pos)

######## Cube 4 #########
    print("Going to Cube 4 Intermediate: \n", cube4_inter_pos)
    arm.safe_move_to_position(cube4_inter_pos)
    print("Going to Cube 4 Final: \n", cube4_pos)
    arm.safe_move_to_position(cube4_pos)
    arm.exec_gripper_cmd(0.03, 50)
    gripper_state = arm.get_gripper_state()
    print("Current State: \n", arm.get_gripper_state())
    gripper_pos = gripper_state['position']
    if (gripper_pos[0] + gripper_pos[1]) < 0.055 and (gripper_pos[0] + gripper_pos[1]) > 0.02:
        COUNT_GRIP = COUNT_GRIP + 1
    print(COUNT_GRIP, " blocks are grasped")
    arm.safe_move_to_position(detect_pos)

    cube4_stack= np.array([[1, 0, 0, 0.562],[0, -1, 0, team_multi*0.2],[0, 0, -1, 0.03+0.05*(COUNT_GRIP - 1)+0.225380201],[0, 0, 0, 1]])
    stack4_target = {'R': cube4_stack[0:3, 0:3], 't': cube4_stack[0:3, 3]}
    stack4_pos = ik.panda_ik(stack4_target)[0]

    print("Stacking Cube 4: \n", stack4_pos)
    arm.safe_move_to_position(stack4_pos)
    arm.exec_gripper_cmd(0.12, 50)
    print("Arm State: \n", arm.get_gripper_state())

    cube4_inter_stack = cube4_stack
    cube4_inter_stack[2,3] = cube4_inter_stack[2,3] + 0.1
    stack4_inter_target = {'R': cube4_inter_stack[0:3, 0:3], 't': cube4_inter_stack[0:3, 3]}
    stack4_inter_pos = ik.panda_ik(stack4_inter_target)[0]
    arm.safe_move_to_position(stack4_inter_pos)

    arm.safe_move_to_position(start_position)

    for i in range(3): # Change as needed:

        #move to scouting position
        #detect_pos = np.array([-pi/2,0,0,-pi/2,0,pi/2,pi/4])
        detect_matrix_dy = np.array([[0,-1,0,0],[-1,0,0,-0.65],[0,0,-1,0.4],[0,0,0,1]])
        detect_target_dy = {'R': detect_matrix_dy[0:3, 0:3], 't': detect_matrix_dy[0:3, 3]}
        detect_pos_dy = ik.panda_ik(detect_target_dy)[0]

        print("Moving to Detection Position")
        print(detect_pos_dy)
        # Open gripper prior to movement to limit timing interference
        arm.exec_gripper_cmd(0.12, 50)
        arm.safe_move_to_position(detect_pos_dy)

        # get the transform from camera to panda_end_effector
        H_ee_camera_dy = detector.get_H_ee_camera()

        # Initialize dynamic cube position
        cube_dynamic = np.eye(4)

        print("Detecting Dynamic Blocks \n")
        name = ''
        FLAG_dy = 0
        while FLAG_dy == 0:
            for (name, pose) in detector.get_detections():
                cube_dynamic = pose
                print(name,'\n',pose)
            if 'cube' in name:
                FLAG_dy = 1

        print("Detection Complete")

        # Get the position and orientation of blocks in world frame
        T0_e_dy = fk.forward(detect_pos_dy)[1]
        print("Matrix of Detection Point: \n", T0_e_dy)
        T0_cube_dynamic = T0_e_dy @ H_ee_camera_dy @ cube_dynamic
        print("Matrix of Dynamic Cube: \n", T0_cube_dynamic)

        # Define omega and time time_delay
        omega = 3.6  # simulation 1.8; real-time 3.6
        time_delay = 7

        # Determine new T of dynamic cube using updated_cube_finder
        cube_dynamic_updated = final.updated_cube_finder(omega, time_delay, T0_cube_dynamic)

        print("Inverse Kinematics for Dynamic Cube 1: \n", cube_dynamic_updated)
        cube_dynamic_target = {'R': cube_dynamic_updated[0:3, 0:3], 't': cube_dynamic_updated[0:3, 3]}
        cube_dynamic_pos = ik.panda_ik(cube_dynamic_target)[0]

        #cube_dynamic_updated = np.array([[1, 0, 0, T0_cube_dynamic[0,3]],[0, -1, 0, T0_cube_dynamic[1,3]],[0, 0, -1, T0_cube_dynamic[2,3]],[0, 0, 0, 1]])
        cube_dynamic_inter = np.zeros([4,4])
        cube_dynamic_inter = cube_dynamic_updated
        cube_dynamic_inter[2, 3] = cube_dynamic_inter[2, 3] + 0.05
        print("Inverse Kinematics for Dynamic Cube Intermediate Location: \n", cube_dynamic_inter)
        cube_dynamic_inter_target = {'R': cube_dynamic_inter[0:3, 0:3], 't': cube_dynamic_inter[0:3, 3]}
        cube_dynamic_inter_pos = ik.panda_ik(cube_dynamic_inter_target)[0]

        # Change stacking height as code becomes more robust

        #print("Going to Dynamic Cube Inter Position: \n", cube_dynamic_inter_pos)
        #arm.safe_move_to_position(cube_dynamic_inter_pos)

        print("Going to Dynamic Cube Position: \n", cube_dynamic_pos)
        arm.safe_move_to_position(cube_dynamic_pos)
        arm.exec_gripper_cmd(0.03, 50)

        gripper_state = arm.get_gripper_state()
        print("Current State: \n", arm.get_gripper_state())
        gripper_pos = gripper_state['position']
        if (gripper_pos[0] + gripper_pos[1]) < 0.055 and (gripper_pos[0] + gripper_pos[1]) > 0.02:
            COUNT_GRIP = COUNT_GRIP + 1

        arm.safe_move_to_position(detect_pos)

        cube_dy_stack= np.array([[1, 0, 0, 0.562],[0, -1, 0, -0.2],[0, 0, -1, 0.035+0.05*(COUNT_GRIP - 1)+0.225380201],[0, 0, 0, 1]])
        stack_dy_target = {'R': cube_dy_stack[0:3, 0:3], 't': cube_dy_stack[0:3, 3]}
        stack_dy_pos = ik.panda_ik(stack_dy_target)[0]

        cube_dy_inter_stack = cube_dy_stack
        cube_dy_inter_stack[2,3] = cube_dy_inter_stack[2,3] + 0.025
        stack_dy_inter_target = {'R': cube_dy_inter_stack[0:3, 0:3], 't': cube_dy_inter_stack[0:3, 3]}
        stack_dy_inter_pos = ik.panda_ik(stack_dy_inter_target)[0]

        print("Moving to Intermediate Stacking Position for Cube: \n", stack1_inter_pos)
        arm.safe_move_to_position(stack1_inter_pos)
        print("Stacking Cube 1: \n", stack1_pos)
        arm.safe_move_to_position(stack1_pos)
        arm.exec_gripper_cmd(0.12, 50)
        print("Arm State: \n", arm.get_gripper_state())

        arm.safe_move_to_position(stack1_inter_pos)

    arm.safe_move_to_position(start_position)

    # END STUDENT CODE
