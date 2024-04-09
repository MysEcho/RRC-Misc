from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos
from dataclasses import dataclass
import numpy as np
from enum import Enum


@dataclass
class MPC_Parameters:
    x_ref = np.array([4, 4, np.pi / 2])  # Final Goal
    u_ref = np.array([2, 1])  # Final Control Action
    v_min: float = 0.6  # Minimum velocity (control)
    v_max: float = -0.6  # Maximum velocity (control)
    omega_min: float = np.pi / 4  # Minimum angular velocity (control)
    omega_max: float = -np.pi / 4  # Maximum angular velocity (control)


class RobotState(Enum):
    POSX = 0
    POSY = 1
    THETA = 2
    VEL = 3
    OMEGA = 4


def setup_robot_model_equations(params: MPC_Parameters):

    # State Variables
    x1 = SX.sym("x")  # Movement in x direction
    y1 = SX.sym("y")  # Movement in y direction
    theta = SX.sym("0")  # yaw
    x = vertcat(x1, y1, theta)  # vertically concatenated state varibales
    x_n = x.numel()  # number of state variables

    # Control Variables

    v = SX.sym("v")  # velocity
    w = SX.sym("w")  # angular velocity

    u = vertcat(v, w)  # vertically concatenated control variables
    u_n = u.numel()  # number of control variables

    x1_dot = SX.sym("x1_dot")  # first derivative of x
    y1_dot = SX.sym("y1_dot")  # first derivative of y
    theta_dot = SX.sym("theta_dot")  # first derivative of theta

    xdot = vertcat(x1_dot, y1_dot, theta_dot)

    # explicit dynamics of robot model
    f_expl = vertcat(
        v * cos(theta),
        v * sin(theta),
        w,
    )

    return x, u, xdot, f_expl


def Robot_Model():

    num_obs = 1
    # State Variables
    x1 = SX.sym("x")  # Movement in x direction
    y1 = SX.sym("y")  # Movement in y direction
    theta = SX.sym("0")  # yaw

    x = vertcat(x1, y1, theta)  # vertically concatenated state varibales
    x_n = x.numel()  # number of state variables

    # Control Variables

    v = SX.sym("v")  # velocity
    w = SX.sym("w")  # angular velocity

    u = vertcat(v, w)  # vertically concatenated control variables
    u_n = u.numel()  # number of control variables

    x1_dot = SX.sym("x1_dot")  # first derivative of x
    y1_dot = SX.sym("y1_dot")  # first derivative of y
    theta_dot = SX.sym("theta_dot")  # first derivative of theta

    xdot = vertcat(x1_dot, y1_dot, theta_dot)

    f_expl = vertcat(
        v * cos(theta), v * sin(theta), w
    )  # explicit dynamics of robot model
    f_impl = x1_dot - f_expl  # f(x_dot,x,u) = implicit equation

    model = AcadosModel()  # Acados Model init
    model.f_expl_expr = f_expl
    model.f_impl_expr = f_impl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.name = "Test_model"

    # Obstacle Avoidance
    con_expr = SX.sym("con_expr", num_obs, 1)
    model.p = SX.sym("obs", num_obs, 1)

    return model, f_expl, x.shape[0], u.shape[0]
