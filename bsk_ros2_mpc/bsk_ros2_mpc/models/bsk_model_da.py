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

def bsk_model_da():
    model = AcadosModel()
    model.name = 'bsk_model_da'

    # parameters
    F_thr = 1.5
    rd_thruster = 0.12
    mass = 17.8
    mass_inv = 1/mass
    inertia = np.diag([0.315]*3)
    inertia_inv = np.linalg.inv(inertia)

    # states
    p = ca.MX.sym('p', 3)
    v = ca.MX.sym('v', 3)
    q = ca.MX.sym('q', 4)
    w = ca.MX.sym('w', 3)
    x = ca.vertcat(p, v, q, w)

    # controls
    u = ca.MX.sym('u', 6)

    # reference
    x_ref = ca.MX.sym('x_ref', x.size()[0])
    u_ref = ca.MX.sym('u_ref', u.size()[0])
    model.p = ca.vertcat(x_ref, u_ref)

    # dynamics
    B_F = F_thr * ca.vertcat(
            ca.horzcat(1., 1., 0., 0., 0., 0.),
            ca.horzcat(0., 0., 1., 1., 0., 0.),
            ca.horzcat(0., 0., 0., 0., 1., 1.)
            )
    B_T = F_thr * rd_thruster * ca.vertcat(
        ca.horzcat(0., 0., 0., 0., 1., -1.),
        ca.horzcat(1., -1., 0., 0., 0., 0.),
        ca.horzcat(0., 0., 1., -1., 0., 0.)
        )
    rotMat = get_rotMat(q)
    w_cross = ca.vertcat(
        ca.horzcat(0, -w[2], w[1]),
        ca.horzcat(w[2], 0, -w[0]),
        ca.horzcat(-w[1], w[0], 0)
    )
    q = q/ca.norm_2(q)
    pdot = v
    vdot = mass_inv * ca.mtimes(rotMat, ca.mtimes(B_F, u))
    qdot = quat_derivative(q, w)
    wdot = ca.mtimes(inertia_inv, ca.mtimes(B_T, u) - ca.mtimes(w_cross, ca.mtimes(inertia, w)))

    xdot = ca.vertcat(pdot, vdot, qdot, wdot)

    # Assign dynamics and controls
    model.f_expl_expr = xdot
    model.x = x
    model.u = u

    # limits
    model.u_min = np.array([-1]*6)
    model.u_max = np.array([1]*6)

    return model