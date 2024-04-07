import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm

# def latexify():
#     params = {
#         "text.latex.preamble": r"\usepackage{gensymb} \usepackage{amsmath}",
#         "axes.labelsize": 10,
#         "axes.titlesize": 10,
#         "legend.fontsize": 10,
#         "xtick.labelsize": 10,
#         "ytick.labelsize": 10,
#         "text.usetex": True,
#         "font.family": "serif",
#     }

#     matplotlib.rcParams.update(params)
# latexify()


def plot_robot(
    dt,
    X_list,
    U_list,
    labels_list,
    X_ref=None,
    U_ref=None,
    u_min=None,
    u_max=None,
    fig_filename=None,
    x_min=None,
    x_max=None,
):

    nx = X_list[0].shape[1]
    nu = U_list[0].shape[1]

    Nsim = U_list[0].shape[0]

    ts = dt * np.arange(0, Nsim + 1)

    states_lables = ["$x$ [m]", "$y$ [m]", "$theta$ [rad]"]
    controls_lables = ["$vel$ [m/s]", "ang_vel[rad/s]"]

    fig, axes = plt.subplots(ncols=2, nrows=nx)

    for i in range(nx):
        for X, label in zip(X_list, labels_list):
            axes[i, 0].plot(ts, X[:, i], label=label, alpha=0.7)

        if X_ref is not None:
            axes[i, 0].step(
                ts,
                X_ref[:, i],
                alpha=0.8,
                where="post",
                label="reference",
                linestyle="dotted",
                color="k",
            )
        axes[i, 0].set_ylabel(states_lables[i])
        axes[i, 0].grid()
        axes[i, 0].set_xlim(ts[0], ts[-1])

        if x_min is not None:
            axes[i, 0].set_ylim(bottom=x_min[i])

        if x_max is not None:
            axes[i, 0].set_ylim(top=x_max[i])

    for i in range(nu):
        for U, label in zip(U_list, labels_list):
            axes[i, 1].step(ts, np.append([U[0, i]], U[:, i]), label=label, alpha=0.7)

        if U_ref is not None:
            axes[i, 1].step(
                ts,
                np.append([U_ref[0, i]], U_ref[:, i]),
                alpha=0.8,
                label="reference",
                linestyle="dotted",
                color="k",
            )
        axes[i, 1].set_ylabel(controls_lables[i])
        axes[i, 1].grid()

        if u_min is not None:
            axes[i, 1].hlines(
                u_max[i], ts[0], ts[-1], linestyles="dashed", alpha=0.8, color="k"
            )
            axes[i, 1].hlines(
                u_min[i], ts[0], ts[-1], linestyles="dashed", alpha=0.8, color="k"
            )
        axes[i, 1].set_xlim(ts[0], ts[-1])

        if u_min is not None:
            axes[i, 1].set_ylim(bottom=0.98 * u_min[i], top=1.02 * u_max[i])

    axes[1, 1].legend(bbox_to_anchor=(0.5, -1.25), loc="lower center")
    axes[-1, 0].set_xlabel("$t$ [min]")
    axes[1, 1].set_xlabel("$t$ [min]")

    fig.delaxes(axes[-1, 1])

    plt.subplots_adjust(
        left=None, bottom=None, right=None, top=None, hspace=0.3, wspace=0.4
    )
    if fig_filename is not None:
        # TODO: legend covers x label :O
        plt.savefig(
            fig_filename, bbox_inches="tight", transparent=True, pad_inches=0.05
        )
        print(f"\nstored figure in {fig_filename}")

    plt.show()


def plot_cstr_iterates(
    dt,
    X_start_list,
    X_end_list,
    U_list,
    X_ref,
    U_ref,
    u_min,
    u_max,
    labels_list,
    fig_filename=None,
    x_min=None,
    x_max=None,
):
    nx = X_start_list[0].shape[1]
    nu = U_list[0].shape[1]

    N_lines = len(U_list)

    Nsim = U_list[0].shape[0]

    ts = dt * np.arange(0, Nsim + 1)

    states_lables = ["$c$ [kmol/m$^3$]", "$T$ [K]", "$h$ [m]"]
    controls_lables = ["$T_c$ [K]", "$F$ [m$^3$/min]"]

    cmap = cm.get_cmap("RdYlBu", N_lines + 2)
    colors = [cmap(i) for i in range(N_lines)]

    fig, axes = plt.subplots(ncols=2, nrows=nx)

    for i in range(nx):
        for X_start, X_end, label, c in zip(
            X_start_list, X_end_list, labels_list, colors
        ):
            for n in range(Nsim):
                axes[i, 0].plot(
                    ts[n : n + 2],
                    [X_start[n, i], X_end[n, i]],
                    label=label,
                    alpha=0.9,
                    color=c,
                )

        axes[i, 0].axhline(
            X_ref[i], label="reference", linestyle="dotted", color="k", alpha=0.8
        )
        axes[i, 0].set_ylabel(states_lables[i])
        axes[i, 0].grid()
        axes[i, 0].set_xlim(ts[0], ts[-1])

        if x_min is not None:
            axes[i, 0].set_ylim(bottom=x_min[i])

        if x_max is not None:
            axes[i, 0].set_ylim(top=x_max[i])

    for i in range(nu):
        for U, label, c in zip(U_list, labels_list, colors):
            axes[i, 1].step(
                ts, np.append([U[0, i]], U[:, i]), label=label, alpha=0.7, color=c
            )

        axes[i, 1].axhline(
            U_ref[i], label="reference", linestyle="dotted", color="k", alpha=0.8
        )
        axes[i, 1].set_ylabel(controls_lables[i])
        axes[i, 1].grid()

        axes[i, 1].axhline(u_max[i], linestyle="dashed", alpha=0.8, color="k")
        axes[i, 1].axhline(u_min[i], linestyle="dashed", alpha=0.8, color="k")
        axes[i, 1].set_xlim(ts[0], ts[-1])
        axes[i, 1].set_ylim(bottom=0.98 * u_min[i], top=1.02 * u_max[i])

    # axes[1, 1].legend(bbox_to_anchor=(0.5, -1.25), loc="lower center")
    axes[-1, 0].set_xlabel("$t$ [min]")
    axes[1, 1].set_xlabel("$t$ [min]")

    fig.delaxes(axes[-1, 1])

    plt.subplots_adjust(
        left=None, bottom=None, right=None, top=None, hspace=0.3, wspace=0.4
    )
    if fig_filename is not None:
        plt.savefig(fig_filename, bbox_inches="tight", transparent=True)
        print(f"\nstored figure in {fig_filename}")

    plt.show()
