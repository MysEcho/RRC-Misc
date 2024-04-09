from acados_template import AcadosOcpSolver, AcadosOcp
import numpy as np
from casadi import SX, vertcat, sin, cos, Function
from model import MPC_Parameters, Robot_Model, setup_robot_model_equations
import time
from integrators import RK4Integrator
from mpc_solver import SolverAcados
from utils import plot_robot, draw_path
import matplotlib.pyplot as plt
from draw_utils import draw


def not_goal_state(x, y) -> bool:
    x_final = MPC_Parameters.x_ref[0]
    y_final = MPC_Parameters.x_ref[1]
    if x_final - x < 0.11 and y_final - y < 0.1:
        return False
    else:
        return True


def simulate_closed_loop(integrator_plant, mpc_solver, x0: np.ndarray, Nsim: int):

    nx = mpc_solver.nx
    nu = mpc_solver.nu
    x = 6
    y = 11

    X = np.ndarray((Nsim + 1, nx))
    U = np.ndarray((Nsim, nu))

    closed_loop_cost = 0

    x_current = x0
    X[[0]] = x_current

    start = time.time()
    # plt.scatter(x, y, marker="x", color="r")
    i = 0
    status = True

    for i in range(Nsim):

        u_current = mpc_solver.solve(x_current)
        x_next = integrator_plant.simulate(x_current, u_current).T
        closed_loop_cost += mpc_solver.eval_stage_cost(x_current, u_current)
        x_current = x_next
        X[[i + 1], :] = x_next
        U[[i], :] = u_current.T
        print(x_next)
        # status = not_goal_state(x_next[0][0], x_next[0][1])
        # print(status)
        # i = i + 1
        # draw_path(x_next[0][0], x_next[0][1])

    closed_loop_cost += mpc_solver.eval_terminal_cost(x_current)
    simulation_time = time.time() - start

    return X, U, simulation_time, closed_loop_cost.item()


def main():
    Nsim = 50
    N = 15
    dt = 0.041

    params = MPC_Parameters()
    xs = params.x_ref
    us = params.u_ref

    x0 = np.array([0, 0, np.pi / 3])

    x, u, xdot, f = setup_robot_model_equations(params)
    Q = 2 * np.diag([5, 5, 0.1])
    R = 2 * np.diag([0.1, 0.01])

    X_all = []
    U_all = []
    labels_all = []
    timings_all = []
    costs_all = []

    # Multiple Shooting
    integrator_plant = RK4Integrator(x, u, f, dt)
    mpc_solver = SolverAcados(x, u, xdot, f, Q, R, xs, us, dt, N)
    X, U, simulation_time, cost = simulate_closed_loop(
        integrator_plant, mpc_solver, x0, Nsim
    )

    X_all.append(X)
    U_all.append(U)
    labels_all.append(f"multiple shooting, acados")
    costs_all.append(cost)
    timings_all.append(simulation_time)
    print(X_all)

    # draw_path(dt, X_all, U_all, xs)
    plot_robot(dt, X_all, U_all, labels_all)
    # draw(X_all)


if __name__ == "__main__":
    main()


"""
POINTERS:

i)Acados OCP solver will give the Optimized solutions for 1 iteration. To make
it an MPC problem, the first (k+1)th solution has to be taken and the OCP has 
to be solved again. This process should continue until the real coordinates of
the robot reaches the goal.

ii) This process will be carried out in the closed loop simulation. The closed loop simulation will take in the current u0 value. 
"""
