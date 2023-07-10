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


class Pendulum_Node_IK: 

    def __init__(self, quasi_sim_manager): 

        self.quasi_sim_manager = quasi_sim_manager
        self.initialise_solver()
        self.angular_velocity = np.zeros(3)
        self.k_gain = 0.25
        self.initialise_pub_sub()   
        self.main_loop()

        rospy.spin()
        

    def callback_conversion(self, msg): 

        self.angular_velocity[0] = self.k_gain*msg.axes[0]
        self.angular_velocity[1] = self.k_gain*msg.axes[1]
        print(self.quasi_sim_manager._current_states[0:7])

    def initialise_solver(self): 

        init_sol = np.zeros(19)
        init_sol[3] = 1
        init_sol[2] = -0.15
        init_sol[9] = -0

        self.quasi_sim_manager.initialise_static_solver_position_boundary(init_sol)
        self.quasi_sim_manager.set_tensions_static_MS_solver_position_boundary([0.0, 0.0, 0])
        self.quasi_sim_manager.solve_static_position_boundary()

        self.quasi_sim_manager.solve_Jacobians_position_boundary()

    def publish_velocities(self): 

        self.velocities_message.value1 = self.quasi_sim_manager._lengths_dot[0]
        self.velocities_message.value2 = self.quasi_sim_manager._lengths_dot[1]
        self.velocities_message.value3 = self.quasi_sim_manager._lengths_dot[2]
        self.velocities_publisher.publish(self.velocities_message)

    def initialise_pub_sub(self): 

        rospy.init_node('Pendulum_Solver_Node', anonymous=True)
        rospy.Subscriber('/joy', Joy, self.callback_conversion)
        self.velocities_publisher = rospy.Publisher('/bulk_set_item', BulkSetItem, queue_size=10)
        self.velocities_message =  BulkSetItem()
        self.velocities_message.id1 = 41
        self.velocities_message.id2 = 42
        self.velocities_message.id3 = 43

    def main_loop(self):

        quasi_sim_manager.apply_tension_differential_position_boundary(np.zeros(3))

        while not rospy.is_shutdown(): 

            tension_input = quasi_sim_manager.solve_differential_inverse_kinematics_position_boundary(self.angular_velocity)
            quasi_sim_manager.apply_tension_differential_position_boundary(tension_input)
            self.publish_velocities()
            time.sleep(1e-2)


tendon_radiuses_list = [[0.0175, 0, 0], [-0.00875, 0.0151554, 0], [-0.00875, -0.0151554, 0]]
tendon_radiuses = SX(tendon_radiuses_list)
robot_arm_1 = Robot_Arm_Params(0.15, 0.05, -0.5, "1", 0.1)
robot_arm_1.from_solid_rod(0.0005, 100e9, 200e9, 8000)
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
diff_inv_solver, _ = diff_inv.create_inverse_differential_kinematics()

initial_solution = np.zeros(19)
initial_solution[3] = 1

init_sol = np.zeros(19)
init_sol[3] = 1
init_sol[2] = -0.15
init_sol[9] = -0

quasi_sim_manager = Quasistatic_Control_Manager(robot_arm_model_1, diff_inv_solver)

Pendulum_Node_IK(quasi_sim_manager)


