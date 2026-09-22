import mujoco
import numpy as np
import matplotlib.pyplot as plt
import matplotlib

matplotlib.use('QtAgg')

class MotorModel:
  def __init__(self, m: mujoco.MjModel, d: mujoco.MjData, motor_name: str, motor_params: dict, ctrl_index: int):
    self.m = m
    self.d = d
    self.motor_params = motor_params
    self.motor_name = motor_name
    self.Kp = motor_params['Kp']
    self.Kd = motor_params['Kd']
    self.gear_ratio = motor_params['gear_ratio']
    self.t_stall = motor_params['stall_torque']
    self.w_no_load = motor_params['no_load_speed']
    self.ctrl_index = ctrl_index
    self.target_pos = 0
    self.target_vel = 0
    self.target_torque = 0
    self.limited_torque = 0
    self.target_vel = 0
    self.tau_max = 0
    self.torques = np.array([])
    self.omegas = np.array([])
    self.target_positions = []
    self.positions = []
    self.velocities = []
    self.time = np.array([])

  def debug(self):
    # print(f"Motor name: {self.motor_name}")
    # print(f"Kp: {self.Kp}")
    # print(f"Kd: {self.Kd}")
    # print(f"Target pos: {self.target_pos}")
    # print(f"Target vel: {self.target_vel}")
    # print(f"Target torque: {self.target_torque}")
    # print(f"Limited torque: {self.limited_torque}")
    # q = self.d.jnt(self.motor_name).qpos
    # qdot = self.d.jnt(self.motor_name).qvel
    # print(f"q: {q}")
    # print(f"qdot: {qdot}")
    if(abs(self.limited_torque)>=abs(self.tau_max)):
      print(f"Motor name: {self.motor_name}")
      print("TORQUE LIMIT REACHED")
      print(f"Target torque: {self.target_torque}")
      print(f"Limited torque: {self.limited_torque}")
      print(f"Angular velocity: {self.d.jnt(self.motor_name).qvel[0]}")

  # Position control function that limits torque according to speed torque curve
  def pos_control(self, target_pos: float):
    self.target_pos = target_pos
    q = self.d.jnt(self.motor_name).qpos
    qdot = self.d.jnt(self.motor_name).qvel
    tau = self.Kp*(self.target_pos - q) - self.Kd*qdot
    self.target_torque = tau
    self.limited_torque = self.speed_torque_limit(tau)
    self.d.ctrl[self.ctrl_index] = self.limited_torque*self.gear_ratio
    return self.limited_torque
  
  # Velocity control function that limits torque according to speed torque curve
  def vel_control(self, target_vel: float):
    self.target_vel = target_vel
    qdot = self.d.jnt(self.motor_name).qvel
    tau = self.Kp*(self.target_vel - qdot) #+ self.Kd*(-self.d.jnt(self.motor_name).qacc)
    self.target_torque = tau
    self.limited_torque = self.speed_torque_limit(tau)

    self.d.ctrl[self.ctrl_index] = self.limited_torque*self.gear_ratio
    return self.limited_torque
  
  def torque_control(self, target_torque: float):
    self.target_torque = target_torque
    self.limited_torque = self.speed_torque_limit(target_torque)
    self.d.ctrl[self.ctrl_index] = self.limited_torque*self.gear_ratio
    return self.limited_torque


  # Function that returns the limited torque according to the speed torque curve
  def speed_torque_limit(self, target_torque: float):
    w_motor = abs(self.d.jnt(self.motor_name).qvel)*self.gear_ratio
    self.tau_max = -(self.t_stall/self.w_no_load)*w_motor + self.t_stall

    # If target torque is greater than speed/torque curve allows, limit it
    if(abs(target_torque) > abs(self.tau_max) and w_motor < abs(self.w_no_load)):
      return np.sign(target_torque)*self.tau_max
    # If angular velocity is greater than no load speed, limit torque to zero
    elif(w_motor >= abs(self.w_no_load)):
      return 0.0
    # Otherwise return the target torque
    else:
      return target_torque


  def log_data(self):
    self.torques = np.append(self.torques,float(abs(self.limited_torque)))
    self.omegas = np.append(self.omegas,float(abs(self.d.jnt(self.motor_name).qvel[0])*self.gear_ratio))

  def log_data_output(self):
    rads_to_rpm = 9.5493
    speed_rpm = abs(self.d.jnt(self.motor_name).qvel[0])*rads_to_rpm
    self.torques = np.append(self.torques,float(abs(self.limited_torque)*self.gear_ratio))
    self.omegas = np.append(self.omegas,float(speed_rpm))


  def plot_data_input(self):
    w_range = np.linspace(0, self.w_no_load, 1000)
    T_line = -self.t_stall/self.w_no_load*w_range + self.t_stall
    plt.title(f"{self.motor_name} Input Shaft Speed Torque Curve")
    plt.plot(self.omegas, self.torques, 'b*')
    plt.plot(w_range, T_line, 'r--')
    plt.xlabel('Angular Velocity (rad/s)')
    plt.ylabel('Torque (Nm)')
    plt.plot(w_range, T_line, 'r--')
    plt.show()
    
  def plot_data_output(self):
    w_range = np.linspace(0, self.w_no_load/self.gear_ratio, 1000)
    stall_torque = self.t_stall*self.gear_ratio
    T_line = -stall_torque/(self.w_no_load/self.gear_ratio)*w_range + stall_torque
    # T_line = -self.t_stall*self.gear_ratio/(self.w_no_load)*w_range + (self.t_stall*self.gear_ratio)
    plt.title(f"{self.motor_name} Input Shaft Speed Torque Curve")
    plt.plot(self.omegas/self.gear_ratio, self.torques*self.gear_ratio, 'b*')
    plt.plot(w_range, T_line, 'r--')
    plt.xlabel('Angular Velocity (rad/s)')
    plt.ylabel('Torque (Nm)')
    plt.plot(w_range, T_line, 'r--')
    plt.show()
    
  def plot_data_output_rpms(self):
    rads_to_rpm = 9.5493
    self.rated_speed = self.motor_params['rated_speed']*rads_to_rpm/self.gear_ratio
    self.rated_torque = self.motor_params['rated_torque']*self.gear_ratio

    #Plot lines for rated continuous speed/torque:
    plt.axhline(y=self.rated_torque, color='g', linestyle='--')
    plt.axvline(x=self.rated_speed, color='g', linestyle='--')
    # plt.text(self.rated_speed*1.1, self.rated_torque*0.9, 'Rated Speed/Torque', color='g')
    rad_to_rpm = 9.5493
    w_range = np.linspace(0, self.w_no_load*rad_to_rpm/self.gear_ratio, 1000)
    stall_torque = self.t_stall*self.gear_ratio
    T_line = -stall_torque/(self.w_no_load*rad_to_rpm/self.gear_ratio)*w_range + stall_torque
    # T_line = -self.t_stall*self.gear_ratio/(self.w_no_load)*w_range + (self.t_stall*self.gear_ratio)
    plt.title(f"{self.motor_name} Input Shaft Speed Torque Curve")
    plt.plot(self.omegas*rad_to_rpm/self.gear_ratio, self.torques*self.gear_ratio, 'b*')
    plt.plot(w_range, T_line, 'r--')
    plt.xlabel('Angular Velocity (rpm)')
    plt.ylabel('Torque (Nm)')
    plt.plot(w_range, T_line, 'r--')
    plt.show()

  def plot_positions(self):
    plt.title(f"{self.motor_name} Position Control")
    plt.plot(self.positions, 'b')
    plt.plot(self.target_positions, 'r')
    plt.xlabel('Time Step')
    plt.ylabel('Position (rad)')
    plt.show()

def plot_motor_data_output(motors_to_plot):
  """
  Plot motor data for specified motors
  :param motors_to_plot: List of motor objects to plot
  """
  plt.ion()
  n_motors = len(motors_to_plot)
  if n_motors == 1:
    nrows = 1
    ncols = 1
  elif n_motors == 2:
    nrows = 1
    ncols = 2
  else:
    nrows = n_motors
    ncols = 1
  fig,axes = plt.subplots(nrows=nrows, ncols=ncols, figsize=(8,2*nrows))
  fig.tight_layout()
  plt.ion()
  i = 0
  for i in range(n_motors):
    motor = motors_to_plot[i]
    axes[i].set_title(f"{motor.motor_name} Input Shaft Speed Torque Curve")
    axes[i].scatter(motor.omegas, motor.torques)
    axes[i].set_xlabel('Angular Velocity (rad/s)')
    axes[i].set_ylabel('Torque (Nm)')

  fig.subplots_adjust(hspace=1,)
  plt.show(block=True)

def log_motor_data(motors_to_plot):
    for motor_obj in motors_to_plot:
        motor_obj.log_data()
    return motors_to_plot

def log_motor_data_output(motors_to_plot):
    for motor_obj in motors_to_plot:
        motor_obj.log_data_output()
    return motors_to_plot