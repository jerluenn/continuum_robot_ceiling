#!/usr/bin/env python
import sys
import numpy as np
import time
from casadi import *
from pyquaternion import Quaternion
import rospy

sys.path.insert(0, "/home/jerluennn/continuum-robot-worm-ceilinginspection/scripts")

from generate_multiple_shooting_solver import Multiple_Shooting_Solver
from generate_robot_arm_model import Robot_Arm_Model
from generate_robot_arm_parameters import Robot_Arm_Params
from quasistatic_control_manager import Quasistatic_Control_Manager
from linear_mpc import Linear_MPC
from dynamixel_sdk_examples.msg import BulkSetItem
from sensor_msgs.msg import Joy


class Pendulum_Node_IK_2: 

    def __init__(self, quasi_sim_manager1, quasi_sim_manager2): 

        self.quasi_sim_manager1 = quasi_sim_manager1
        self.quasi_sim_manager2 = quasi_sim_manager2
        self.initialise_solver()
        self.angular_velocity = np.zeros(3)
        self.angular_velocity2 = np.zeros(3)
        self.k_gain = 0.5
        self.k_gain_2 = 0.5
        self.initialise_pub_sub()   
        self.main_loop()

        rospy.spin()
        
    def callback_conversion(self, msg): 

        self.angular_velocity[0] = self.k_gain*msg.axes[0]
        self.angular_velocity[1] = self.k_gain*msg.axes[1]
        self.angular_velocity2[0] = self.k_gain_2*msg.axes[3]
        self.angular_velocity2[1] = self.k_gain_2*msg.axes[4]
        print("1: ", self.quasi_sim_manager1._current_states[0:7])
        print("===================================")
        print("2: ", self.quasi_sim_manager2._current_states[0:7])

    def initialise_solver(self): 

        init_sol = np.zeros(19)
        init_sol[3] = 1
        init_sol[2] = -0.15
        init_sol[9] = -0

        self.quasi_sim_manager1.initialise_static_solver_position_boundary(init_sol)
        self.quasi_sim_manager1.set_tensions_static_MS_solver_position_boundary([0.0, 0.0, 0])
        self.quasi_sim_manager1.solve_static_position_boundary()

        self.quasi_sim_manager1.solve_Jacobians_position_boundary()

        self.quasi_sim_manager2.initialise_static_solver_position_boundary(init_sol)
        self.quasi_sim_manager2.set_tensions_static_MS_solver_position_boundary([0.0, 0.0, 0])
        self.quasi_sim_manager2.solve_static_position_boundary()

        self.quasi_sim_manager2.solve_Jacobians_position_boundary()

    def publish_velocities(self): 

        vel = np.vstack((self.quasi_sim_manager1._lengths_dot, self.quasi_sim_manager2._lengths_dot))

        vel = np.where(vel<0, vel, vel*0.2)

        self.velocities_message.value1 = vel[0]
        self.velocities_message.value2 = vel[1]
        self.velocities_message.value3 = vel[2]
        self.velocities_message.value4 = vel[3]
        self.velocities_message.value5 = vel[4]
        self.velocities_message.value6 = vel[5]
        self.velocities_publisher.publish(self.velocities_message)

    def publish_velocities_tension(self, tension1, tension2): 

        # print(tension1, tension2)
        self.velocities_message.value1 = tension1[0]
        self.velocities_message.value2 = tension1[1]
        self.velocities_message.value3 = tension1[2]
        self.velocities_message.value4 = tension2[0]
        self.velocities_message.value5 = tension2[1]
        self.velocities_message.value6 = tension2[2]
        self.velocities_publisher.publish(self.velocities_message)        

    def initialise_pub_sub(self): 

        rospy.init_node('Pendulum_Solver_Node', anonymous=True)
        rospy.Subscriber('/joy', Joy, self.callback_conversion)
        self.velocities_publisher = rospy.Publisher('/bulk_set_item', BulkSetItem, queue_size=10)
        self.velocities_message =  BulkSetItem()
        self.velocities_message.id1 = 41
        self.velocities_message.id2 = 42
        self.velocities_message.id3 = 43
        self.velocities_message.id4 = 12
        self.velocities_message.id5 = 11
        self.velocities_message.id6 = 13
        

    def main_loop(self):

        quasi_sim_manager1.apply_tension_differential_position_boundary(np.zeros(3))
        quasi_sim_manager2.apply_tension_differential_position_boundary(np.zeros(3))

        while not rospy.is_shutdown(): 

            tension_input = quasi_sim_manager1.solve_differential_inverse_kinematics_position_boundary(self.angular_velocity)
            quasi_sim_manager1.apply_tension_differential_position_boundary(tension_input)
            tension_input2 = quasi_sim_manager2.solve_differential_inverse_kinematics_position_boundary(self.angular_velocity2)
            quasi_sim_manager2.apply_tension_differential_position_boundary(tension_input2)
            self.publish_velocities()
            time.sleep(1e-2)


tendon_radiuses_list = [[0.02*1.4, 0, 0], [-0.00875*1.4, 0.0151554*1.4, 0], [-0.00875*1.4, -0.0151554*1.4, 0]]
tendon_radiuses = SX(tendon_radiuses_list)
robot_arm_1 = Robot_Arm_Params(0.25, 0.05, -0.5, "1", 0.2)
robot_arm_1.from_solid_rod(0.0005, 200e9, 100e9, 8000)
robot_arm_1.set_gravity_vector('-z')
C = np.diag([0.000, 0.000, 0.000])
Bbt = np.diag([1e-4, 1e-4, 1e-4])
Bse = Bbt
# Bse = np.zeros((3,3))
# Bbt = np.zeros((3,3))
robot_arm_1.set_damping_coefficient(C)
robot_arm_1.set_damping_factor(Bbt, Bse)
robot_arm_1.set_tendon_radiuses(tendon_radiuses_list)
robot_arm_model_1 = Robot_Arm_Model(robot_arm_1)

Q_w_p = 100e3*np.eye(4)
Q_w_t = 0.5e1*np.eye(3)
n_states = 4
n_tendons = 3
name = 'single_controller'
R_mat = 1e-2*np.eye(3)
Tf = 0.01
q_max = 40
q_dot_max = 5

diff_inv = Linear_MPC(Q_w_p, Q_w_t, n_states, n_tendons,q_dot_max, q_max, name, R_mat, Tf)
diff_inv_solver1, _ = diff_inv.create_inverse_differential_kinematics()

robot_arm_2 = Robot_Arm_Params(0.25, 0.05, -0.5, "1", 0.0)
robot_arm_2.from_solid_rod(0.0005, 200e9, 100e9, 8000)
robot_arm_2.set_gravity_vector('-z')
robot_arm_2.set_damping_coefficient(C)
robot_arm_2.set_damping_factor(Bbt, Bse)
robot_arm_2.set_tendon_radiuses(tendon_radiuses_list)
robot_arm_model_2 = Robot_Arm_Model(robot_arm_2)

Q_w_p = 100e3*np.eye(4)
Q_w_t = 0.5e1*np.eye(3)
n_states = 4
n_tendons = 3
name = 'single_controller'
R_mat = 1e-2*np.eye(3)
Tf = 0.01
q_max = 40
q_dot_max = 5

diff_inv2 = Linear_MPC(Q_w_p, Q_w_t, n_states, n_tendons,q_dot_max, q_max, name, R_mat, Tf)
diff_inv_solver2, _ = diff_inv2.create_inverse_differential_kinematics()

quasi_sim_manager1 = Quasistatic_Control_Manager(robot_arm_model_1, diff_inv_solver1)
quasi_sim_manager2 = Quasistatic_Control_Manager(robot_arm_model_2, diff_inv_solver2)

Pendulum_Node_IK_2(quasi_sim_manager1, quasi_sim_manager2)


