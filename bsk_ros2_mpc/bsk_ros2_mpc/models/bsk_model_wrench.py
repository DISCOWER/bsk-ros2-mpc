#!/usr/bin/env python
__author__ = "Elias Krantz"
__contact__ = "eliaskra@kth.se"

import casadi as ca
import numpy as np
from acados_template import AcadosModel

def get_rotMat(q):
    rotMat = ca.vertcat(
            ca.horzcat(1-2*q[2]**2-2*q[3]**2, 2*(q[1]*q[2]-q[0]*q[3]), 2*(q[1]*q[3]+q[0]*q[2])),
            ca.horzcat(2*(q[1]*q[2]+q[0]*q[3]), 1-2*q[1]**2-2*q[3]**2, 2*(q[2]*q[3]-q[0]*q[1])),
            ca.horzcat(2*(q[1]*q[3]-q[0]*q[2]), 2*(q[2]*q[3]+q[0]*q[1]), 1-2*q[1]**2-2*q[2]**2)
    )
    return rotMat

def quat_mult(q1, q2):
    return ca.vertcat(
            q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3],
            q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2],
            q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1],
            q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]
        )

def quat_derivative(q, w):
    return 0.5 * quat_mult(q, ca.vertcat(0, w))

def bsk_model_wrench(n_others=0):
    model = AcadosModel()
    model.name = 'bsk_model_wrench'

    # parameters
    mass = 17.8
    mass_inv = 1/mass
    inertia = np.diag([0.315]*3)
    inertia_inv = np.linalg.inv(inertia)

    d_min_sqr = (0.5 * 1)**2

    # states
    p = ca.MX.sym('p', 3)
    v = ca.MX.sym('v', 3)
    q = ca.MX.sym('q', 4)
    w = ca.MX.sym('w', 3)
    x = ca.vertcat(p, v, q, w)

    # controls
    u = ca.MX.sym('u', 6)
    
    q = q / ca.norm_2(q)
    rotMat = get_rotMat(q)
    w_cross = ca.vertcat(
        ca.horzcat(0, -w[2], w[1]),
        ca.horzcat(w[2], 0, -w[0]),
        ca.horzcat(-w[1], w[0], 0)
    )

    F = ca.vertcat(u[0], u[1], u[2])
    T = ca.vertcat(u[3], u[4], u[5])

    pdot = v
    vdot = mass_inv * ca.mtimes(rotMat, F)
    qdot = quat_derivative(q, w)
    wdot = ca.mtimes(inertia_inv, (T - ca.mtimes(w_cross, ca.mtimes(inertia, w))))

    xdot = ca.vertcat(pdot, vdot, qdot, wdot)

    # Assign dynamics and controls
    model.f_expl_expr = xdot
    model.x = x
    model.u = u

    # limits
    F_lim = 2 * 1.5 * 2/3
    T_lim = 2 * 0.17 * 1.5 * 1/3
    model.u_min = np.array([-F_lim, -F_lim, -F_lim, -T_lim, -T_lim, -T_lim])
    model.u_max = np.array([F_lim, F_lim, F_lim, T_lim, T_lim, T_lim])

    # parameters: other agents' positions at each timestep
    x_ref = ca.MX.sym('x_ref', x.size()[0])
    u_ref = ca.MX.sym('u_ref', u.size()[0])
    p_others = ca.MX.sym('p_others', 3 * n_others)

    # combine parameters
    model.p = ca.vertcat(x_ref, u_ref, p_others)

    # Add collision avoidance constraints
    dist_constraints = []

    # Collision constraints with obstacles
    for i in range(n_others):
        dist = ca.sumsqr(x[:3] - p_others[3*i:3*(i+1)])
        dist_constraints.append(d_min_sqr - dist)

    model.con_h_expr = ca.vertcat(*dist_constraints)
    model.con_h_expr_e = ca.vertcat(*dist_constraints)

    return model