import numpy as np
import casadi as ca
from acados_template import AcadosModel
from acados_template import AcadosOcp, AcadosOcpSolver
import scipy


class SolverAcados:

    def __init__(
        self,
        x: ca.MX,
        u: ca.MX,
        xdot: ca.MX,
        f: ca.MX,
        Q: np.ndarray,
        R: np.ndarray,
        xs: np.ndarray,
        us: np.ndarray,
        dt: float,
        N: int,
    ):
        num_obs = 1  # Number of Obstacles
        obs_param = ca.SX.sym("obs", num_obs, 1)
        con_expr = ca.SX.sym("con_expr", num_obs, 1)
        self.nx = x.shape[0]
        self.nu = u.shape[0]

        # this is just needed for the closed-loop simulation
        l = (x - xs).T @ Q @ (x - xs) + (u - us).T @ R @ (u - us)
        E = (x - xs).T @ Q @ (x - xs)
        self.l_fun = ca.Function("stage_cost", [x, u], [l])
        self.E_fun = ca.Function("terminal_cost", [x], [E])

        # setting up the acados model
        model = AcadosModel()

        f_impl = xdot - f
        model.f_impl_expr = f_impl
        model.f_expl_expr = f
        model.x = x
        model.xdot = xdot
        model.u = u
        # p_obs = [obs_param[2 * k : 2 * (k + 1)] for k in range(0, num_obs)]
        # for i in range(0, num_obs):
        #     dist = x[0:2] - p_obs[i]
        #     con_expr[i] = ca.sqrt(ca.sumsqr(dist))
        # model.con_h_expr = con_expr
        # model.con_h_expr_e = con_expr
        # model.p = obs_param
        model.name = "robot"

        self.model = model

        # setting up the acados OCP
        ocp = AcadosOcp()

        ocp.model = self.model
        ocp.dims.N = N
        ocp.solver_options.tf = dt

        # set cost

        ocp.cost.cost_type = "EXTERNAL"
        ocp.cost.cost_type_e = "EXTERNAL"

        ocp.model.cost_expr_ext_cost = (x - xs).T @ Q @ (x - xs) + (u - us).T @ R @ (
            u - us
        )
        ocp.model.cost_expr_ext_cost_e = (x - xs).T @ Q @ (x - xs)

        # SET CONSTRAINTS

        """
        Velocity Constraints
        """
        # Input Constraints
        min_forward_velocity = float(-10.0)
        max_forward_velocity = float(15.0)
        min_angular_velocity = float(-30.0)
        max_angular_velocity = float(20.0)
        ocp.constraints.lbu = np.array(
            [min_forward_velocity, min_angular_velocity]
        )  # [min_forward_velocity,min_angular_velocity]
        ocp.constraints.ubu = np.array(
            [max_forward_velocity, max_angular_velocity]
        )  # [max_forward_velocity,max_angular_velocity]
        ocp.constraints.idxbu = np.array(range(2))

        # Obstacle Avoidance Constrainsts
        # ocp.dims.np = 2
        # obs_radius = np.array([5.0])
        # print("np=", ocp.dims.np)
        # ocp.parameter_values = np.zeros((num_obs,))  # Initialize Parameters
        # ocp.constraints.lh = obs_radius
        # ocp.constraints.uh = 1e2 * np.ones((num_obs,))
        # ocp.constraints.lh_e = obs_radius
        # ocp.constraints.uh_e = 1e2 * np.ones((num_obs,))

        # Set Initial State
        ocp.constraints.x0 = np.array([2, 2, 0])

        # set options
        ocp.solver_options.qp_solver = (
            "FULL_CONDENSING_QPOASES"  # FULL_CONDENSING_QPOASES
        )
        # PARTIAL_CONDENSING_HPIPM, FULL_CONDENSING_QPOASES, FULL_CONDENSING_HPIPM,
        ocp.solver_options.qp_solver_cond_N = N  # for partial condensing

        # ocp.solver_options.print_level = 1
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.nlp_solver_type = "SQP"  # SQP_RTI, SQP
        ocp.solver_options.levenberg_marquardt = 1e-5

        # integrator options
        ocp.solver_options.integrator_type = "IRK"
        ocp.solver_options.sim_method_num_stages = 4
        ocp.solver_options.sim_method_num_steps = 1
        ocp.solver_options.sim_method_newton_iter = 10

        simX = np.zeros((N + 1, self.nx))
        simU = np.zeros((N, self.nu))

        self.solver = AcadosOcpSolver(ocp, json_file="acados_ocp.json")
        # for i in range(N):
        #     simX[i, :] = self.solver.get(i, "x")
        #     simU[i, :] = self.solver.get(i, "u")
        # simX[N, :] = self.solver.get(N, "x")
        # print(simX)
        # set initial guess
        # for n in range(N):
        #     self.solver.set(n, "x", xs)
        #     self.solver.set(n, "u", us)
        # self.solver.set(N, "x", xs)
        # self.solver.print_statistics()

    def solve(self, x0):
        u0 = self.solver.solve_for_x0(x0.ravel(), fail_on_nonzero_status=False)
        return u0

    def eval_stage_cost(self, x, u):
        return self.l_fun(x, u).full()

    def eval_terminal_cost(self, x):
        return self.E_fun(x).full()
