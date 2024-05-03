import pybullet as p
import time
import math
import numpy as np
from math import *
import keyboard
from enum import Enum


DEG2RAD = math.pi/180.0 
RAD2DEG = 180.0/math.pi
ref_elbow = 30*DEG2RAD
ref_shoulder = 20*DEG2RAD
init_position = (0,80,90,0,0,0,0,0,0,0,0)
reset_position = (0,0,0,0,0,0,0,0,0,0,0)

max_base_joint_angle = 100
max_shoulder_joint_angle = 150
max_elbow_joint_angle = 150
max_rot_joint_angle =  180
max_tilt_joint_angle = 200

min_base_joint_angle = -100
min_shoulder_joint_angle = 0
min_elbow_joint_angle = 0
min_rot_joint_angle = -180
min_tilt_joint_angle = 0

max_base_joint_angle_rad = max_base_joint_angle * DEG2RAD
max_shoulder_joint_angle_rad = max_shoulder_joint_angle * DEG2RAD
max_elbow_joint_angle_rad = max_elbow_joint_angle * DEG2RAD
max_rot_joint_angle_rad =  max_rot_joint_angle * DEG2RAD
max_tilt_joint_angle_rad = max_tilt_joint_angle * DEG2RAD

min_base_joint_angle_rad = min_base_joint_angle * DEG2RAD
min_shoulder_joint_angle_rad = min_shoulder_joint_angle * DEG2RAD
min_elbow_joint_angle_rad = min_elbow_joint_angle * DEG2RAD
min_rot_joint_angle_rad = min_rot_joint_angle * DEG2RAD
min_tilt_joint_angle_rad = min_tilt_joint_angle * DEG2RAD

class State(Enum):
    BEGIN = 1
    INIT = 2
    CHOOSE_MODE = 3
    INIT_DK = 4 
    INIT_IK = 5
    MODE_DK = 6
    MODE_IK = 7
    RESULT = 8
    INIT_EMMERGENCY = 9
    EMMERGENCY = 10
    FINISH = 11

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

        self.result_base_joint_angle = 0
        self.result_shoulder_joint_angle = 0
        self.result_elbow_joint_angle = 0
        self.result_rot_joint_angle = 0
        self.result_tilt_joint_angle = 0
        self.result_speed = 0


        # init joint 
        self.base_joint_angle = 0.0
        self.shoulder_joint_angle = 0.0
        self.elbow_joint_angle = 0.0
        self.rot_joint_angle = 0.0 
        self.tilt_joint_angle = 0.0

        self.verification_apply = False
        # self.verification_init = False

        #State of the system
        self._state = State.BEGIN


    def init_start (self) :
        press_button_init = p.readUserDebugParameter(self.init_button) - self.last_value_button_init
        if keyboard.is_pressed('i') or press_button_init == 1 :
            self.base_joint_angle = 0.0*DEG2RAD
            self.shoulder_joint_angle = -80*DEG2RAD + ref_shoulder
            self.elbow_joint_angle = - 90*DEG2RAD + ref_elbow
            self.rot_joint_angle = 0.0*DEG2RAD 
            self.tilt_joint_angle = 0.0*DEG2RAD
            self.move_joint()
            elbow_new_ref = self.elbow_joint_angle - ref_elbow
            shoulder_new_ref = self.shoulder_joint_angle - ref_shoulder
            self.result(0)
            self.last_base_joint_angle = self.base_joint_angle
            self.last_shoulder_joint_angle = shoulder_new_ref
            self.last_elbow_joint_angle = elbow_new_ref
            self.last_rot_joint_angle = self.rot_joint_angle
            self.last_tilt_joint_angle = self.tilt_joint_angle
            self.last_value_button_init = 0
            p.removeAllUserParameters()
            self.mode_dk_button = p.addUserDebugParameter(" Mode : Angle  ", 1, 0, 0)
            self.mode_ik_button = p.addUserDebugParameter(" Mode : Position ", 1, 0, 0)
            self._state = State.CHOOSE_MODE

    def init_dk(self) :
        # Sliders des joints
        p.removeAllUserParameters()
        self.base_slider = p.addUserDebugParameter("Base Joint", min_base_joint_angle, max_base_joint_angle, self.base_joint_angle*RAD2DEG)
        self.shoulder_slider = p.addUserDebugParameter("Shoulder Joint", min_shoulder_joint_angle, max_shoulder_joint_angle, -self.shoulder_joint_angle*RAD2DEG + ref_shoulder*RAD2DEG)
        self.elbow_slider = p.addUserDebugParameter("Elbow Joint", min_elbow_joint_angle, max_elbow_joint_angle, -self.elbow_joint_angle*RAD2DEG + ref_elbow*RAD2DEG)
        self.rot_slider = p.addUserDebugParameter("Rot Joint", min_rot_joint_angle, max_rot_joint_angle, self.rot_joint_angle*RAD2DEG)
        self.tilt_slider = p.addUserDebugParameter("Tilt Joint", min_tilt_joint_angle, max_tilt_joint_angle, self.tilt_joint_angle*RAD2DEG)
        self.speed_slider = p.addUserDebugParameter(" Speed ", 0, 1, 0)
        self.apply_button = p.addUserDebugParameter(" Apply ", 1, 0, 0)
        self.init_button = p.addUserDebugParameter(" Init ", 1, 0, 0)
        self.reset_button = p.addUserDebugParameter(" Reset ", 1, 0, 0)
        self.mode_ik_button = p.addUserDebugParameter(" Mode : Position ", 1, 0, 0)
        self.stop_button = p.addUserDebugParameter(" STOP ", 1, 0, 0)

    def init_ik(self) :
        p.removeAllUserParameters()
        self.x_slider = p.addUserDebugParameter(" X ", -100, 100, self.base_joint_angle*RAD2DEG) # valeur position du joint 
        self.y_slider = p.addUserDebugParameter(" Y ", 0, 150, -self.shoulder_joint_angle*RAD2DEG + ref_shoulder*RAD2DEG)
        self.z_slider = p.addUserDebugParameter(" Z ", 0, 150, -self.elbow_joint_angle*RAD2DEG + ref_elbow*RAD2DEG)
        self.roll_slider = p.addUserDebugParameter(" Roll ", -180, 180, self.rot_joint_angle*RAD2DEG)
        self.pitch_slider = p.addUserDebugParameter(" Pitch", 0, 200, self.tilt_joint_angle*RAD2DEG)
        self.yaw_slider = p.addUserDebugParameter(" Yaw ", 0, 150, 0)
        self.speed_slider = p.addUserDebugParameter(" Speed ", 0, 1, 0)
        self.apply_button = p.addUserDebugParameter(" Apply ", 1, 0, 0)
        self.init_button = p.addUserDebugParameter(" Init ", 1, 0, 0)
        self.reset_button = p.addUserDebugParameter(" Reset ", 1, 0, 0)
        self.mode_dk_button = p.addUserDebugParameter(" Mode : Angle ", 1, 0, 0)
        self.stop_button = p.addUserDebugParameter(" STOP ", 1, 0, 0)

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
    
    def rec_position(self,value) : 
        # print(value)
        p.removeAllUserParameters()
        self.base_slider = p.addUserDebugParameter("Base Joint", min_base_joint_angle, max_base_joint_angle, value[0])
        self.shoulder_slider = p.addUserDebugParameter("Shoulder Joint", min_shoulder_joint_angle, max_shoulder_joint_angle, value[1])
        self.elbow_slider = p.addUserDebugParameter("Elbow Joint", min_elbow_joint_angle, max_elbow_joint_angle, value[2])
        self.rot_slider = p.addUserDebugParameter("Rot Joint", min_rot_joint_angle, max_rot_joint_angle, value[3])
        self.tilt_slider = p.addUserDebugParameter("Tilt Joint", min_tilt_joint_angle, max_tilt_joint_angle, value[4])
        self.speed_slider = p.addUserDebugParameter(" Speed ", 0, 1, value[5])
        self.apply_button = p.addUserDebugParameter(" Apply ", 1, 0, value[6])
        self.init_button = p.addUserDebugParameter(" Init ", 1, 0, value[7])
        self.reset_button = p.addUserDebugParameter(" Reset ", 1, 0, value[8])
        self.mode_ik_button = p.addUserDebugParameter(" Mode : Position ", 1, 0, value[9])
        self.stop_button = p.addUserDebugParameter(" STOP ", 1, 0, value[10])
        self.last_value_button_apply = 0

    def go_to_init(self) : 
        
        press_button_init = p.readUserDebugParameter(self.init_button) - self.last_value_button_init

        if keyboard.is_pressed('i') or press_button_init == 1 :
            print("start init ............................")
            self.rec_position(init_position)
            # self.verification_init = True
            time.sleep(0.2)
            return 1

    def go_to_reset(self) :
        press_button_reset = p.readUserDebugParameter(self.reset_button) - self.last_value_button_reset

        if keyboard.is_pressed('i') or press_button_reset == 1 :
            print("start reset ............................")
            self.rec_position(reset_position)
            time.sleep(0.2)

    def position_zero(self) :
        self.base_joint_angle = 0.0*DEG2RAD
        self.shoulder_joint_angle = -0.0*DEG2RAD + ref_shoulder
        self.elbow_joint_angle = -0.0*DEG2RAD + ref_elbow
        self.rot_joint_angle = 0.0*DEG2RAD 
        self.tilt_joint_angle = 0.0*DEG2RAD

    def aplly_value(self) : 
        press_button_apply = p.readUserDebugParameter(self.apply_button)- self.last_value_button_apply
        if keyboard.is_pressed('a') or press_button_apply == 1 :
            print("start apply ............................")
            self.aplly_value_dk()
            time.sleep(0.2)

    def aplly_value_dk(self) : 
        elbow_new_ref = self.elbow_joint_angle - ref_elbow
        shoulder_new_ref = self.shoulder_joint_angle - ref_shoulder
        self.result(p.readUserDebugParameter(self.speed_slider))
        self.last_base_joint_angle = self.base_joint_angle
        self.last_shoulder_joint_angle = shoulder_new_ref
        self.last_elbow_joint_angle = elbow_new_ref
        self.last_rot_joint_angle = self.rot_joint_angle
        self.last_tilt_joint_angle = self.tilt_joint_angle
        self.last_value_button_apply = p.readUserDebugParameter(self.apply_button)
        self.last_value_button_init = p.readUserDebugParameter(self.init_button)
        self.last_value_button_reset = p.readUserDebugParameter(self.reset_button)
        self.last_value_button_stop = p.readUserDebugParameter(self.stop_button) 

    def constrain_min_max(self, val, min_val, max_val):
        return min(max_val, max(min_val, val))

    def constrain_axe(self) : 
        elbow_new_ref = self.elbow_joint_angle - ref_elbow
        shoulder_new_ref = self.shoulder_joint_angle - ref_shoulder

        max_rot = abs(elbow_new_ref) * 3
        if abs(self.rot_joint_angle) > max_rot  and  not self.rot_joint_angle == 0:
            # print((self.rot_joint_angle/abs(self.rot_joint_angle)))
            self.rot_joint_angle=max_rot*(self.rot_joint_angle/abs(self.rot_joint_angle))

        max_tilt = abs( elbow_new_ref )*2.7 + ( exp(abs(self.rot_joint_angle))*0.3 -0.3) +  90 * DEG2RAD
        # print(max_tilt)
        if self.tilt_joint_angle > max_tilt  : 
            self.tilt_joint_angle = max_tilt

        max_shoulder =  150*DEG2RAD +abs(elbow_new_ref) - abs(self.tilt_joint_angle) * 0.2 -0.5
        # print(max_shoulder )
        # print(shoulder_new_ref)
        if (shoulder_new_ref)*(-1) > max_shoulder   :
            self.shoulder_joint_angle = (max_shoulder - ref_shoulder)*(-1)
        
        # print(self.shoulder_joint_angle*RAD2DEG)
        # print(self.elbow_joint_angle*RAD2DEG)
        self.base_joint_angle = self.constrain_min_max(self.base_joint_angle,min_base_joint_angle_rad,max_base_joint_angle_rad)
        self.shoulder_joint_angle = -self.constrain_min_max(self.shoulder_joint_angle*-1,min_shoulder_joint_angle_rad - ref_shoulder,max_shoulder_joint_angle_rad - ref_shoulder)
        self.elbow_joint_angle = -self.constrain_min_max(self.elbow_joint_angle*-1,min_elbow_joint_angle_rad - ref_elbow,max_elbow_joint_angle_rad - ref_elbow)
        self.rot_joint_angle = self.constrain_min_max(self.rot_joint_angle,min_rot_joint_angle_rad,max_rot_joint_angle_rad)
        self.tilt_joint_angle = self.constrain_min_max(self.tilt_joint_angle,min_tilt_joint_angle_rad,max_tilt_joint_angle_rad)

        # print(self.shoulder_joint_angle*RAD2DEG)
        # print(self.elbow_joint_angle*RAD2DEG)

            
    def move_joint(self) : 
        p.resetJointState(self.robot_id, self.base_joint1_index, targetValue=self.base_joint_angle, targetVelocity = 0)
        p.resetJointState(self.robot_id, self.joint1_joint2_index, targetValue=self.shoulder_joint_angle, targetVelocity = 0)
        p.resetJointState(self.robot_id, self.joint3_joint4_index, targetValue=self.elbow_joint_angle, targetVelocity = 0)
        p.resetJointState(self.robot_id, self.joint4_joint5_index, targetValue=self.rot_joint_angle, targetVelocity = 0)
        p.resetJointState(self.robot_id, self.joint5_joint6_index, targetValue=self.tilt_joint_angle, targetVelocity = 0)

    def read_joint_dk(self) :
        self.base_joint_angle = p.readUserDebugParameter(self.base_slider)*DEG2RAD
        self.shoulder_joint_angle = -p.readUserDebugParameter(self.shoulder_slider)*DEG2RAD + ref_shoulder
        self.elbow_joint_angle = - p.readUserDebugParameter(self.elbow_slider)*DEG2RAD + ref_elbow
        self.rot_joint_angle = p.readUserDebugParameter(self.rot_slider)*DEG2RAD 
        self.tilt_joint_angle = p.readUserDebugParameter(self.tilt_slider)*DEG2RAD

    def read_joint_ik(self) :
        self.x = p.readUserDebugParameter(self.x_slider)
        self.y = -p.readUserDebugParameter(self.y_slider)
        self.z = - p.readUserDebugParameter(self.z_slider)
        self.roll = p.readUserDebugParameter(self.roll_slider) 
        self.pitch = p.readUserDebugParameter(self.pitch_slider)
        self.yaw = p.readUserDebugParameter(self.yaw_slider)

    def mode_dk(self):

        # Lecture sliders

        self.read_joint_dk()
    
        # print(base_joint_angle)
        # print(shoulder_joint_angle-ref_shoulder)
        # print(elbow_joint_angle-ref_elbow)
        # print(rot_joint_angle)
        # print(tilt_joint_angle)
        self.constrain_axe()
        self.go_to_init()
        self.go_to_reset()
        # print(base_joint_angle)
        # print((shoulder_joint_angle)*(-1))
        # print((elbow_joint_angle-last_elbow_joint_angle - 1.57)*(-1))
        # print(rot_joint_angle-last_rot_joint_angle)
        # print(tilt_joint_angle)
        # print((tilt_joint_angle-self.last_tilt_joint_angle)*(-1))
        # print(abs(max_tilt))
        # print(p.readUserDebugParameter(self.apply_button))
        # print(self.last_value_button_apply)
        self.aplly_value()
        if keyboard.is_pressed('2') or p.readUserDebugParameter(self.mode_ik_button) == 1 :
            print("start init ............................")
            self._state=State.INIT_IK
            time.sleep(0.2)

        if p.readUserDebugParameter(self.stop_button) == 1 :
            print("start emergency ............................")
            self._state=State.INIT_EMMERGENCY
            time.sleep(0.2)

    def mode_ik(self) : 
        
        self.read_joint_ik()
        if keyboard.is_pressed('1') or p.readUserDebugParameter(self.mode_dk_button) == 1 :
            print("start init ............................")
            self._state=State.INIT_DK
            time.sleep(0.2)

        if p.readUserDebugParameter(self.stop_button) == 1 :
            print("start emergency ............................")
            self._state=State.INIT_EMMERGENCY
            time.sleep(0.2)

    def run_simulation(self):
        """Callback function triggered every 100 milliseconds to execute to robot program"""
        # print(self._state)
        self.verification_apply = False
        # self.verification_init = False
        # print(self.verification_init)
        if self._state == State.BEGIN:
            p.removeAllUserParameters()
            self.position_zero()
            self.move_joint()
            self.init_button = p.addUserDebugParameter(" Init ", 1, 0, 0)
            self._state = State.INIT
        elif self._state == State.INIT :
            self.init_start()
        elif self._state == State.CHOOSE_MODE :
            if p.readUserDebugParameter(self.mode_dk_button) == 1 or keyboard.is_pressed('1') :
                self._state = State.INIT_DK 
            elif p.readUserDebugParameter(self.mode_ik_button) == 1 or keyboard.is_pressed('2') :
                self._state = State.INIT_IK
        elif self._state == State.INIT_DK: 
            self.init_dk()
            self._state = State.MODE_DK
        elif self._state == State.INIT_IK:
            self.init_ik()
            self._state = State.MODE_IK
        elif self._state == State.MODE_DK: 
            self.mode_dk()
            self.draw_axes_all()  
            self.move_joint()
        elif self._state ==  State.MODE_IK:
            self.mode_ik() 
            self.draw_axes_all()  
            self.move_joint() 
        elif self._state == State.RESULT:
            self._state = State.GO_HOME 
        elif self._state == State.INIT_EMMERGENCY:
            p.removeAllUserParameters()
            self.position_zero()
            self.move_joint()
            self.restart_button = p.addUserDebugParameter(" Restart ", 1, 0, 0)
            self._state = State.EMMERGENCY       
        elif self._state == State.EMMERGENCY:
            if p.readUserDebugParameter(self.restart_button) == 1 :
                print("start emergency ............................")
                self._state=State.BEGIN   
                time.sleep(0.2) 
        elif self._state == State.FINISH:
            self.go_to_reset()
            
        p.stepSimulation()
        time.sleep(1.0 / 500.0)

    def result(self,speed) : 
        elbow_new_ref = self.elbow_joint_angle - ref_elbow
        shoulder_new_ref = self.shoulder_joint_angle - ref_shoulder
        self.result_base_joint_angle = (self.base_joint_angle-self.last_base_joint_angle)
        self.result_shoulder_joint_angle = (shoulder_new_ref-self.last_shoulder_joint_angle)*(-1)
        self.result_elbow_joint_angle = (elbow_new_ref-self.last_elbow_joint_angle)*(-1)
        self.result_rot_joint_angle = (self.rot_joint_angle-self.last_rot_joint_angle)*(-1)
        self.result_tilt_joint_angle = (self.tilt_joint_angle-self.last_tilt_joint_angle)
        self.result_speed = speed
        self.verification_apply = True

    def get_dk_angle(self) :
        return self.result_base_joint_angle, self.result_shoulder_joint_angle, self.result_elbow_joint_angle, self.result_rot_joint_angle, self.result_tilt_joint_angle , self.result_speed

    def close_simulation(self):
        if keyboard.is_pressed('esc'):
            win.close()
            p.disconnect()
