############################################################################
#
#   Copyright (C) 2024 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Elias Krantz"
__contact__ = "eliaskra@kth.se"

import numpy as np
import os
from scipy.linalg import block_diag
import casadi as ca
from acados_template import AcadosOcp, AcadosOcpSolver
from ..models.bsk_model_wrench import bsk_model_wrench

class MpcFollowerWrench():
    def __init__(self, n_others=0):
        # Define the controller parameters
        self.dt = 0.2               # MPC time step [s]
        self.Nx = 30                # Prediction horizon, states             
        self.Q = np.diag([          # State weighting matrix
            1e0, 1e0, 1e0,
            3e1, 3e1, 3e1, 
            5e3, 
            5e1, 5e1, 5e1])           
        self.R = np.diag([          # State weighting matrix
            2e-1, 2e-1, 2e-1,
            5e2, 5e2, 5e2]) 
        self.P = 20 * self.Q        # Terminal state weighting matrix

        # Initial state (position, velocity, quaternion, angular velocity)
        self.x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # Number of other agents
        self.n_others = n_others

        # Weight on slack varibles for collision avoidance
        self.W_slack = np.array([1e5]*self.n_others)
        # Indexes of slack variables
        self.idx_slack = np.arange(len(self.W_slack))
        # Indexes of slack variables related to collision avoidance
        self.idxsh = np.arange(self.n_others)

        # Create the OCP
        self.solver = self.setup()

    def setup(self):
        # create ocp object to formulate the OCP
        ocp = AcadosOcp()
        
        # Set directory for code generation and json file
        this_file_dir = os.path.dirname(os.path.abspath(__file__))
        package_root = os.path.abspath(os.path.join(this_file_dir, '..'))
        codegen_dir = os.path.join(package_root, 'mpc_codegen')
        json_path = os.path.join(codegen_dir, 'acados_ocp.json')
        os.makedirs(codegen_dir, exist_ok=True)
        ocp.code_export_directory = codegen_dir

        # Define the model
        model = bsk_model_wrench(n_others=self.n_others)
        ocp.model = model

        # Set dimensions
        nx = model.x.size()[0]
        nu = model.u.size()[0]
        self.nx = nx
        self.nu = nu
        ocp.parameter_values = np.zeros(model.p.size()[0])

        # Get variables
        x_ref = model.p[:nx]
        u_ref = model.p[nx:nx+nu]

        # Define the cost function
        ocp.cost.cost_type = 'NONLINEAR_LS'
        ocp.cost.cost_type_e = 'NONLINEAR_LS'

        # Set the weighting matrices
        ocp.cost.W = block_diag(self.Q, self.R)
        ocp.cost.W_e = block_diag(self.P)

        quat_error = 1 - (model.x[6:10].T @ x_ref[6:10])**2
        ocp.model.cost_y_expr = ca.vertcat(
            model.x[0:3] - x_ref[0:3],   # Position error
            model.x[3:6] - x_ref[3:6],   # Velocity error
            quat_error,
            model.x[10:13] - x_ref[10:13],  # Angular velocity error
            model.u - u_ref, # Control error
        )
        # Terminal cost 
        ocp.model.cost_y_expr_e = ca.vertcat(
            model.x[0:3] - x_ref[0:3],
            model.x[3:6] - x_ref[3:6],
            quat_error,
            model.x[10:13] - x_ref[10:13],
        )
        ocp.cost.yref = np.zeros(ocp.model.cost_y_expr.shape[0])  # Reference for full cost function
        ocp.cost.yref_e = np.zeros(ocp.model.cost_y_expr_e.shape[0])  # Terminal reference

        # Set constraints on U
        ocp.constraints.lbu = model.u_min
        ocp.constraints.ubu = model.u_max
        ocp.constraints.idxbu = np.arange(nu)

        # Set up the constraints for collision avoidance
        ocp.constraints.lh = np.full(self.n_others, -1e9)   # lower bounds on con_h_expr
        ocp.constraints.uh = np.zeros(self.n_others)  # no upper bounds (one-sided constraint)  
        ocp.constraints.idxsh = self.idxsh  # index of slack variables corresponding to con_h_expr
        ocp.constraints.lh_e = np.full(self.n_others, -1e9)   # lower bounds on con_h_expr
        ocp.constraints.uh_e = np.zeros(self.n_others)  # no upper bounds (one-sided constraint)  
        ocp.constraints.idxsh_e = self.idxsh  # index of slack variables corresponding to con_h_expr

        # Set slack variables
        ocp.cost.Zl = self.W_slack
        ocp.cost.Zu = self.W_slack
        ocp.cost.zl = np.zeros(self.idx_slack.size)
        ocp.cost.zu = np.zeros(self.idx_slack.size)
        ocp.cost.Zl_e = self.W_slack
        ocp.cost.Zu_e = self.W_slack
        ocp.cost.zl_e = np.zeros(self.idx_slack.size)
        ocp.cost.zu_e = np.zeros(self.idx_slack.size)

        # Set initial state
        ocp.constraints.x0 = self.x0

        # Set the prediction horizon
        ocp.solver_options.N_horizon = self.Nx
        ocp.solver_options.tf = self.dt * self.Nx

        # Set the solver options
        ocp.solver_options.nlp_solver_type = "SQP_RTI"
        ocp.solver_options.nlp_solver_max_iter = 1
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.print_level = 0

        ocp_solver = AcadosOcpSolver(ocp, json_file=json_path)
        return ocp_solver

    def get_input(self, x0, x_ref, x_others=[]):
        # Properly set x0, i.e. constrain it to x0
        self.solver.set(0, "lbx", x0.flatten())
        self.solver.set(0, "ubx", x0.flatten())

        n_others = len(x_others)
        if n_others > self.n_others:
            raise ValueError(f"Number of other agents {n_others} exceeds the maximum number {self.n_others}.")

        # Update reference
        if x_ref.ndim == 1:
            x_ref = x_ref.reshape(-1, 1)
        len_x_ref = x_ref.shape[1]

        # loop over time steps
        for k in range(self.Nx + 1):
            if k < len_x_ref:
                p_k = x_ref[:,k]
            else:
                p_k = x_ref[:,-1]

            # reference input
            u_ref_k = np.zeros(self.nu)
            p_k = np.concatenate((p_k, u_ref_k), axis=0)

            # position other agents
            for i in range(n_others):
                p_k = np.concatenate((p_k, x_others[i][k]), axis=0)
            for i in range(self.n_others - n_others):
                # Handle missing agents by adding a large position
                pos_other = x0[:3].flatten() + np.array([1e2]*3)
                p_k = np.concatenate((p_k, pos_other), axis=0)

            self.solver.set(k, "p", p_k)

        status = self.solver.solve()
        u_opt = self.solver.get(0, 'u')
        if status != 0:
            print("Solver failed. Retrying with warm-start reset.")
            self.solver.reset()
            status = self.solver.solve()
            if status != 0:
                print("Solver failed again. Using previous solution.")
                u_opt =  self.solver.get(1, 'u')
        
        # get solution
        x_pred = np.ndarray((self.Nx+1, self.nx))
        for i in range(self.Nx):
            x_pred[i,:] = self.solver.get(i, "x")
        x_pred[self.Nx,:] = self.solver.get(self.Nx, "x")

        # u_opt[:3][np.abs(u_opt[:3]) < 1e-2] = 0.0
        # u_opt[3:][np.abs(u_opt[3:]) < 1e-3] = 0.0
        # Alternatively, use indices to avoid view assignment issues:
        
        return u_opt, x_pred