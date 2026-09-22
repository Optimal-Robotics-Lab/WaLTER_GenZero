import time
import mujoco
import mujoco.viewer
import numpy as np
import pygame
import walter_genzero.lib.MotorModel as motor

class JoystickController:
    def __init__(self, controller_type: str, m: mujoco.MjModel, d: mujoco.MjData, motors: motor.MotorModel):
        self.controller_type = controller_type
        self.m = m
        self.d = d
        self.motors = motors

        # Initializing joystick with pygame
        pygame.init()
        self.js = pygame.joystick.Joystick(0)
        self.js.init()
        print(self.js.get_name())

        # User control variables:
        self.joystick_deadzone = 0.2
        self.max_wheel_vel = 100
        self.max_knee_vel = 0.01

        # Desired setpoints
        self.fr_knee_des_pos = 0
        self.fl_knee_des_pos = 0
        self.br_knee_des_pos = 0
        self.bl_knee_des_pos = 0

        self.fr_knee_des_vel = 0
        self.fl_knee_des_vel = 0
        self.br_knee_des_vel = 0
        self.bl_knee_des_vel = 0

        self.fr_hip_des_pos = 0
        self.fl_hip_des_pos = 0
        self.br_hip_des_pos = 0
        self.bl_hip_des_pos = 0

        self.fr_hip_des_vel = 0
        self.fl_hip_des_vel = 0
        self.br_hip_des_vel = 0
        self.bl_hip_des_vel = 0

        self.right_wheel_vel_des = 0
        self.left_wheel_vel_des = 0

        self.d_up = 0
        self.d_down = 0
        self.d_left = 0
        self.d_right = 0

        self.hip_splay = 0
        self.dt = m.opt.timestep


    # Function that controls the wheels
    def control_wheels(self):
        # Control the wheels with basic joystick control
        self.left_wheel_vel_des = self.max_wheel_vel*(-self.left_stick_ud + self.left_stick_lr)
        self.right_wheel_vel_des = self.max_wheel_vel*(-self.left_stick_ud - self.left_stick_lr)

    def control_knees(self):
        self.left_knee_des_vel = self.max_knee_vel*(-self.right_stick_ud + self.right_stick_lr)
        self.right_knee_des_vel = self.max_knee_vel*(-self.right_stick_ud - self.right_stick_lr)
        self.fl_knee_des_pos += self.left_knee_des_vel
        self.bl_knee_des_pos += self.left_knee_des_vel
        self.fr_knee_des_pos += self.right_knee_des_vel
        self.br_knee_des_pos += self.right_knee_des_vel
        


    # Function that handles button inputs
    def button_controls(self):
        if(self.a_button):
            self.fr_hip_des_pos = 0
            self.fl_hip_des_pos = 0
            self.br_hip_des_pos = 0
            self.bl_hip_des_pos = 0
            self.fr_knee_des_pos = self.nearest_pi(self.fr_knee_pos)
            self.fl_knee_des_pos = self.nearest_pi(self.fl_knee_pos)
            self.br_knee_des_pos = self.nearest_pi(self.br_knee_pos)
            self.bl_knee_des_pos = self.nearest_pi(self.bl_knee_pos)
        elif(self.b_button):
            self.fr_hip_des_pos = -np.pi/2
            self.fl_hip_des_pos = -np.pi/2
            self.br_hip_des_pos = np.pi/2
            self.bl_hip_des_pos = np.pi/2
            self.fr_knee_des_pos = self.nearest_pi(self.fr_knee_pos)
            self.fl_knee_des_pos = self.nearest_pi(self.fl_knee_pos)
            self.br_knee_des_pos = self.nearest_pi(self.br_knee_pos)
            self.bl_knee_des_pos = self.nearest_pi(self.bl_knee_pos)
        elif(self.y_button):
            self.fr_hip_des_pos = -np.pi/3
            self.fl_hip_des_pos = -np.pi/3
            self.br_hip_des_pos = np.pi/3
            self.bl_hip_des_pos = np.pi/3
        elif(self.x_button):
            self.fr_hip_des_pos = -np.pi/12
            self.fl_hip_des_pos = -np.pi/12
            self.br_hip_des_pos = np.pi/12
            self.bl_hip_des_pos = np.pi/12
            self.fr_knee_des_pos = self.nearest_pi(self.fr_knee_pos) - np.pi/4
            self.fl_knee_des_pos = self.nearest_pi(self.fl_knee_pos) - np.pi/4
            self.br_knee_des_pos = self.nearest_pi(self.br_knee_pos) + np.pi/4
            self.bl_knee_des_pos = self.nearest_pi(self.bl_knee_pos) + np.pi/4


    def update_hip_splay(self):
        self.hip_splay = 0.001*self.d_up
        self.front_splay = -0.001*self.left_bumper
        self.fr_hip_des_pos -= self.hip_splay + self.front_splay
        self.fl_hip_des_pos -= self.hip_splay + self.front_splay
        self.br_hip_des_pos += self.hip_splay
        self.bl_hip_des_pos += self.hip_splay


    # Main control function
    def control(self,m,d):
        # Get current joint angles
        self.get_mujoco_state(m,d)

        # Get joystick state
        self.get_joystick_state()
        # Reset the simulation if requested
        if(self.start_button):
            mujoco.mj_resetData(m, d)
            self.fr_knee_des_pos = 0
            self.fl_knee_des_pos = 0
            self.br_knee_des_pos = 0
            self.bl_knee_des_pos = 0
            self.fr_hip_des_pos = 0
            self.fl_hip_des_pos = 0
            self.br_hip_des_pos = 0
            self.bl_hip_des_pos = 0

        self.update_hip_splay()

        # Control the wheels
        self.control_wheels()

        # Control the knees
        self.control_knees()

        # Apply button controls
        self.button_controls()

        # Finally send the commands to the motors
        self.send_commands()

        return m,d


    # Function that sends the commands to the motors
    def send_commands(self):
        self.motors[0].pos_control(self.fr_hip_des_pos)
        self.motors[1].pos_control(self.fl_hip_des_pos)
        self.motors[2].pos_control(self.br_hip_des_pos)
        self.motors[3].pos_control(self.bl_hip_des_pos)

        self.motors[4].pos_control(self.fr_knee_des_pos)
        self.motors[5].pos_control(self.fl_knee_des_pos)
        self.motors[6].pos_control(self.br_knee_des_pos)
        self.motors[7].pos_control(self.bl_knee_des_pos)

        self.motors[9].vel_control(self.right_wheel_vel_des)
        self.motors[8].vel_control(self.right_wheel_vel_des)
        self.motors[10].vel_control(self.left_wheel_vel_des)
        self.motors[11].vel_control(self.left_wheel_vel_des)
        self.motors[12].vel_control(self.right_wheel_vel_des)
        self.motors[13].vel_control(self.right_wheel_vel_des)
        self.motors[14].vel_control(self.left_wheel_vel_des)
        self.motors[15].vel_control(self.left_wheel_vel_des)

            

    # Calculates nearest pi value to the current angle
    def nearest_pi(self, current_angle):
        near_pi = np.round(current_angle/np.pi)*np.pi
        return near_pi
    
    # Get joystick state
    def get_joystick_state(self):
        pygame.event.get()
        if(self.controller_type == "logitech"):
            self.left_stick_lr = self.js.get_axis(0)
            self.left_stick_ud = self.js.get_axis(1)
            self.left_trigger = self.js.get_axis(2)
            self.right_stick_lr = self.js.get_axis(3)
            self.right_stick_ud = self.js.get_axis(4)
            self.right_trigger = self.js.get_axis(5)
            self.a_button = self.js.get_button(0)
            self.b_button = self.js.get_button(1)
            self.x_button = self.js.get_button(2)
            self.y_button = self.js.get_button(3)
            self.left_bumper = self.js.get_button(4)
            self.right_bumper = self.js.get_button(5)
            self.start_button = self.js.get_button(7)
            self.back_button = self.js.get_button(6)
            self.home_button = self.js.get_button(8)
            self.d_up = self.js.get_hat(0)[1]
            self.d_down = self.js.get_hat(0)[1]
            self.d_left = self.js.get_hat(0)[0]
            self.d_right = self.js.get_hat(0)[0]
            
        elif(self.controller_type == "logitech2"):
            self.left_stick_lr = self.js.get_axis(0)
            self.left_stick_ud = self.js.get_axis(1)
            # self.left_trigger = self.js.get_axis(2)
            self.right_stick_lr = self.js.get_axis(2)
            self.right_stick_ud = self.js.get_axis(3)+0.16
            # self.right_trigger = self.js.get_axis(5)
            self.a_button = self.js.get_button(0)
            self.b_button = self.js.get_button(1)
            self.x_button = self.js.get_button(2)
            self.y_button = self.js.get_button(3)
            self.start_button = self.js.get_button(7)
            self.back_button = self.js.get_button(6)
            self.home_button = self.js.get_button(8)
            self.d_up = self.js.get_hat(0)[1]
            self.d_down = self.js.get_hat(0)[1]
            self.d_left = self.js.get_hat(0)[0]
            self.d_right = self.js.get_hat(0)[0]
            self.left_bumper = self.js.get_button(4)
            self.right_bumper = self.js.get_button(5)

        elif(self.controller_type == "ps4"):
            self.left_stick_lr = self.js.get_axis(0)
            self.left_stick_ud = self.js.get_axis(1)
            self.left_trigger = self.js.get_axis(4)
            self.right_stick_lr = self.js.get_axis(2)
            self.right_stick_ud = self.js.get_axis(3)
            self.right_trigger = self.js.get_axis(5)
            self.a_button = self.js.get_button(0)
            self.b_button = self.js.get_button(1)
            self.x_button = self.js.get_button(2)
            self.y_button = self.js.get_button(3)

            self.aux_button = self.js.get_button(4)
            self.start_button = self.js.get_button(6)

            self.d_up = self.js.get_button(11)
            self.d_down = self.js.get_button(12)
            # self.d_left = self.js.get_button(13)
            # self.d_right = self.js.get_button(14)

            self.left_bumper = self.js.get_button(9)
            self.right_bumper = self.js.get_button(10)

        # Filter joystick values based on deadzone:
        if(abs(self.left_stick_lr) < self.joystick_deadzone):
            self.left_stick_lr = 0
        if(abs(self.left_stick_ud) < self.joystick_deadzone):
            self.left_stick_ud = 0
        if(abs(self.right_stick_lr) < self.joystick_deadzone):
            self.right_stick_lr = 0
        if(abs(self.right_stick_ud) < self.joystick_deadzone):
            self.right_stick_ud = 0

    # Get current joint angles
    def get_mujoco_state(self,m,d):
        # Get current joint angles
        self.fr_hip_pos = self.d.jnt('head_right_thigh_joint').qpos[0]
        self.fl_hip_pos = self.d.jnt('head_left_thigh_joint').qpos[0]
        self.br_hip_pos = self.d.jnt('torso_right_thigh_joint').qpos[0]
        self.bl_hip_pos = self.d.jnt('torso_right_thigh_joint').qpos[0]

        self.fr_knee_pos = self.d.jnt('head_right_thigh_shin_joint').qpos[0]
        self.fl_knee_pos = self.d.jnt('head_left_thigh_shin_joint').qpos[0]
        self.br_knee_pos = self.d.jnt('torso_right_thigh_shin_joint').qpos[0]
        self.bl_knee_pos = self.d.jnt('torso_left_thigh_shin_joint').qpos[0]



    def print_all_joystick_states(self):
        self.get_joystick_state()
        # Print all joystick button states:

        print("A Button:", self.a_button)
        print("B Button:", self.b_button)
        print("X Button:", self.x_button)
        print("Y Button:", self.y_button)

        # print("Left Bumper:", self.left_bumper)
        # print("Right Bumper:", self.right_bumper)
        print("Index 1: ", self.js.get_button(1))
        print("Index 2: ", self.js.get_button(2))
        print("Index 3: ", self.js.get_button(3))
        
        
        print("Index 4: ", self.js.get_button(4))
        print("Index 5: ", self.js.get_button(5))
        print("Index 6: ", self.js.get_button(6))
        print("Index 7: ", self.js.get_button(7))
        print("Index 8: ", self.js.get_button(8))
        print("Index 9: ", self.js.get_button(9))
        print("Index 10: ", self.js.get_button(10))
        # print("Index 11: ", self.js.get_button(11))
        # print("Index 12: ", self.js.get_button(12))
        # print("Index 13: ", self.js.get_button(13))
        # print("Index 14: ", self.js.get_button(14))

