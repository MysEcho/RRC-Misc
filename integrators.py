import numpy as np
import casadi as ca


class ExplicitEulerIntegrator:

    def __init__(self, x: ca.MX, u: ca.MX, f: ca.MX, dt: float, num_steps: int = 1):

        self.f_continuous_fun = ca.Function("f", [x, u], [f])

        self.dt = dt
        self.num_steps = num_steps

        # setup the integrator
        h = dt / num_steps
        x_next = x

        for _ in range(num_steps):
            k = self.f_continuous_fun(x_next, u)
            x_next = x_next + h * k

        self.f_discrete_fun = ca.Function("f_discrete", [x, u], [x_next])

    def simulate(self, x0: np.ndarray, u0: np.ndarray):
        return self.f_discrete_fun(x0, u0).full()


class RK4Integrator:

    def __init__(self, x: ca.MX, u: ca.MX, f: ca.MX, dt: float, num_steps: int = 1):

        self.f_continuous_fun = ca.Function("f", [x, u], [f])

        self.dt = dt
        self.num_steps = num_steps

        h = dt / num_steps
        x_next = x

        for _ in range(num_steps):

            k_1 = self.f_continuous_fun(x_next, u)
            k_2 = self.f_continuous_fun(x_next + 0.5 * h * k_1, u)
            k_3 = self.f_continuous_fun(x_next + 0.5 * h * k_2, u)
            k_4 = self.f_continuous_fun(x_next + k_3 * h, u)

            x_next = x_next + h * (1 / 6) * (k_1 + 2 * k_2 + 2 * k_3 + k_4)

        self.f_discrete_fun = ca.Function("f_discrete", [x, u], [x_next])

    def simulate(self, x0: np.ndarray, u0: np.ndarray):
        return self.f_discrete_fun(x0, u0).full()


class CVODESIntegrator:

    def __init__(self, x: ca.MX, u: ca.MX, f: ca.MX, dt: float):

        self.f_continuous_fun = ca.Function("f", [x, u], [f])

        self.dt = dt

        ode = {"x": x, "p": u, "ode": f}

        t0 = 0.0
        tf = dt
        x_next = ca.integrator("F", "cvodes", ode, t0, tf)(x0=x, p=u)["xf"]

        self.f_discrete_fun = ca.Function("f_discrete", [x, u], [x_next])

    def simulate(self, x0: np.ndarray, u0: np.ndarray):
        return self.f_discrete_fun(x0, u0).full()
