import pybullet as p
import time
import math
import numpy as np
from math import *
import keyboard



DEG2RAD = math.pi/180.0 
RAD2DEG = 180.0/math.pi



class SimulationPyBullet:
    def __init__(self):
        self.physics_client = p.connect(p.GUI)
        self.flag_axe = 0

        p.resetDebugVisualizerCamera(1.0, 90, -40, (0, 0, 0))

        p.setGravity(0,0,-9.81)
        p.setRealTimeSimulation(0)

        p.loadURDF("plane.urdf",[0,0,0],[0,0,0,1])
        # self.file_urdf = p.loadURDF("R6A_visual_3d.urdf",[0,0,0],[0,0,0,1],useFixedBase = True)
        self.file_urdf = p.loadURDF("R6A_visual_3d_new.urdf",[0,0,0],[0,0,0,1],useFixedBase = True)

        self.robot_id= self.file_urdf
         
        # les 6 axes
        self.base_joint1_index = 0
        self.joint1_joint2_index = 1
        self.joint3_joint4_index = 2
        self.joint4_joint5_index = 3
        self.joint5_joint6_index = 4

        # anciennne valeur
        self.last_base_joint_angle = 0.0
        self.last_shoulder_joint_angle = 0.0
        self.last_elbow_joint_angle = 0.0
        self.last_rot_joint_angle = 0.0
        self.last_tilt_joint_angle = 0.0
        self.last_value_button_apply = 0
        self.last_value_button_init = 0
        self.last_value_button_reset = 0
        self.last_value_button_stop = 0

        # result angle cinematique direct

        self.dk_base_joint_angle = 0
        self.dk_shoulder_joint_angle = 0
        self.dk_elbow_joint_angle = 0
        self.dk_rot_joint_angle = 0
        self.dk_tilt_joint_angle = 0

        # Sliders des joints
        self.base_slider = p.addUserDebugParameter("Base Joint", -100, 100, 0.0)
        self.shoulder_slider = p.addUserDebugParameter("Shoulder Joint", 0, 150, 0.0)
        self.elbow_slider = p.addUserDebugParameter("Elbow Joint", 0, 150, 0.0)
        self.rot_slider = p.addUserDebugParameter("Rot Joint", -180, 180, 0.0)
        self.tilt_slider = p.addUserDebugParameter("Tilt Joint", 0, 200, 0.0)
        self.apply_button = p.addUserDebugParameter(" Apply ", 1, 0, 0)
        self.init_button = p.addUserDebugParameter(" Init ", 1, 0, 0)
        self.reset_button = p.addUserDebugParameter(" Reset ", 1, 0, 0)
        self.stop_button = p.addUserDebugParameter(" STOP ", 1, 0, 0)

        #State of the system
        self._state = State.BEGIN

    def rec_position(self,value) : 
        # print(value)
        p.removeAllUserParameters()
        self.base_slider = p.addUserDebugParameter("Base Joint", -100, 100, value[0])
        self.shoulder_slider = p.addUserDebugParameter("Shoulder Joint", 0, 150, value[1])
        self.elbow_slider = p.addUserDebugParameter("Elbow Joint", 0, 150, value[2])
        self.rot_slider = p.addUserDebugParameter("Rot Joint", -180, 180, value[3])
        self.tilt_slider = p.addUserDebugParameter("Tilt Joint", 0, 200, value[4])
        self.apply_button = p.addUserDebugParameter(" Apply ", 1, 0, value[5])
        self.init_button = p.addUserDebugParameter(" Init ", 1, 0, value[6])
        self.reset_button = p.addUserDebugParameter(" Reset ", 1, 0, value[7])
        self.stop_button = p.addUserDebugParameter(" STOP ", 1, 0, value[8])
        self.last_value_button_apply = 0

    def aplly_value (self,base_joint_angle,shoulder_new_ref,elbow_new_ref,rot_joint_angle,tilt_joint_angle):
        self.dk_base_joint_angle = (base_joint_angle-self.last_base_joint_angle)
        self.dk_shoulder_joint_angle = (shoulder_new_ref-self.last_shoulder_joint_angle)*(-1)
        self.dk_elbow_joint_angle = (elbow_new_ref-self.last_elbow_joint_angle)*(-1)
        self.dk_rot_joint_angle = (rot_joint_angle-self.last_rot_joint_angle)*(-1)
        self.dk_tilt_joint_angle = (tilt_joint_angle-self.last_tilt_joint_angle)
        self.last_base_joint_angle = base_joint_angle
        self.last_shoulder_joint_angle = shoulder_new_ref
        self.last_elbow_joint_angle = elbow_new_ref
        self.last_rot_joint_angle = rot_joint_angle
        self.last_tilt_joint_angle = tilt_joint_angle
        self.last_value_button_apply = p.readUserDebugParameter(self.apply_button)
        self.last_value_button_init = p.readUserDebugParameter(self.init_button)
        self.last_value_button_reset = p.readUserDebugParameter(self.reset_button)
        self.last_value_button_stop = p.readUserDebugParameter(self.stop_button)
        

    def draw_axes(self,index) :
        joint_info= p.getJointInfo(self.robot_id,index)
        joint_pose = p.getLinkState(self.robot_id, joint_info[0])[:2]
        joint_pose = p.getLinkState(self.robot_id, joint_info[0])[:2]

        # Dessiner les axes du joint
        length = 0.1  # Longueur des axes
        x_axis = np.array([length, 0, 0])
        y_axis = np.array([0, length, 0])
        z_axis = np.array([0, 0, length])

        p.addUserDebugLine(joint_pose[0], joint_pose[0] + x_axis, [1, 0, 0], 2, 1)
        p.addUserDebugLine(joint_pose[0], joint_pose[0] + y_axis, [0, 1, 0], 2, 1)
        p.addUserDebugLine(joint_pose[0], joint_pose[0] + z_axis, [0, 0, 1], 2, 1)


    def draw_axes_all(self):
        if keyboard.is_pressed('r') and self.flag_axe == 1:
            self.flag_axe = 0
        if keyboard.is_pressed('s') or self.flag_axe == 1 :
           
            self.draw_axes(self.base_joint1_index )
            self.draw_axes(self.joint1_joint2_index)
            self.draw_axes(self.joint3_joint4_index)
            self.draw_axes(self.joint4_joint5_index)
            self.draw_axes(self.joint5_joint6_index)
            self.flag_axe = 1


    def run_simulation(self):
        init_position = (0,90,90,0,0,0,0,0,0)
        reset_position = (0,0,0,0,0,0,0,0,0)
         
        ref_elbow = 30*DEG2RAD
        ref_shoulder = 30*DEG2RAD

        # Lecture sliders
        base_joint_angle = p.readUserDebugParameter(self.base_slider)*DEG2RAD
        shoulder_joint_angle = -p.readUserDebugParameter(self.shoulder_slider)*DEG2RAD + ref_shoulder
        elbow_joint_angle = - p.readUserDebugParameter(self.elbow_slider)*DEG2RAD + ref_elbow
        rot_joint_angle = p.readUserDebugParameter(self.rot_slider)*DEG2RAD 
        tilt_joint_angle = p.readUserDebugParameter(self.tilt_slider)*DEG2RAD

        
        
    
        # print(base_joint_angle)
        # print(shoulder_joint_angle-ref_shoulder)
        # print(elbow_joint_angle-ref_elbow)
        # print(rot_joint_angle)
        # print(tilt_joint_angle)
     
     

        elbow_new_ref = elbow_joint_angle - ref_elbow
        shoulder_new_ref = shoulder_joint_angle - ref_shoulder



        max_rot = abs(elbow_new_ref) * 3
        if abs(rot_joint_angle) > max_rot  and  not rot_joint_angle == 0:
            # print((rot_joint_angle/abs(rot_joint_angle)))
            rot_joint_angle=max_rot*(rot_joint_angle/abs(rot_joint_angle))


        max_tilt = abs( elbow_new_ref )*2.7 + ( exp(abs(rot_joint_angle))*0.3 -0.3) +  90 * DEG2RAD
        # print(max_tilt)
        if tilt_joint_angle > max_tilt  : 
            tilt_joint_angle = max_tilt


        max_shoulder =  150*DEG2RAD +abs(elbow_new_ref) - abs(tilt_joint_angle) * 0.2 -0.5

        # print(max_shoulder )
        # print(shoulder_new_ref)
        if (shoulder_new_ref)*(-1) > max_shoulder   :
            shoulder_joint_angle = (max_shoulder - ref_shoulder)*(-1)
            shoulder_new_ref = shoulder_joint_angle - ref_shoulder


        # print(base_joint_angle)
        # print((shoulder_joint_angle)*(-1))
        # print((elbow_joint_angle-last_elbow_joint_angle - 1.57)*(-1))
        # print(rot_joint_angle-last_rot_joint_angle)
        # print(tilt_joint_angle)
        # print((tilt_joint_angle-self.last_tilt_joint_angle)*(-1))
        # print(abs(max_tilt))
        # print(p.readUserDebugParameter(self.apply_button))
        # print(self.last_value_button_apply)

        press_button_init = p.readUserDebugParameter(self.init_button) - self.last_value_button_init
        if keyboard.is_pressed('i') or press_button_init == 1 :
            print("start init ............................")
            self.rec_position(init_position)
            time.sleep(0.2)


        press_button_reset = p.readUserDebugParameter(self.reset_button) - self.last_value_button_reset
        if keyboard.is_pressed('i') or press_button_reset == 1 :
            print("start reset ............................")
            self.rec_position(reset_position)
            time.sleep(0.2)

        press_button_apply = p.readUserDebugParameter(self.apply_button)- self.last_value_button_apply
        if keyboard.is_pressed('a') or press_button_apply == 1 :
            print("start apply ............................")
            self.aplly_value (base_joint_angle,shoulder_new_ref,elbow_new_ref,rot_joint_angle,tilt_joint_angle)  
            time.sleep(0.2)

        self.draw_axes_all()

        p.resetJointState(self.robot_id, self.base_joint1_index, targetValue=base_joint_angle, targetVelocity = 0)
        p.resetJointState(self.robot_id, self.joint1_joint2_index, targetValue=shoulder_joint_angle, targetVelocity = 0)
        p.resetJointState(self.robot_id, self.joint3_joint4_index, targetValue=elbow_joint_angle, targetVelocity = 0)
        p.resetJointState(self.robot_id, self.joint4_joint5_index, targetValue=rot_joint_angle, targetVelocity = 0)
        p.resetJointState(self.robot_id, self.joint5_joint6_index, targetValue=tilt_joint_angle, targetVelocity = 0)
        
        p.stepSimulation()
        time.sleep(1.0 / 500.0)
    


    def get_dk_angle(self) :
        return self.dk_base_joint_angle, self.dk_shoulder_joint_angle, self.dk_elbow_joint_angle, self.dk_rot_joint_angle, self.dk_tilt_joint_angle

    def close_simulation(self):
        if keyboard.is_pressed('esc'):
            win.close()
            p.disconnect()
