#!/usr/bin/env python

import numpy as np
import os
import time
from scipy.linalg import block_diag
import casadi as ca
from acados_template import AcadosOcp, AcadosOcpSolver
from ..models.bsk_model_wrench import bsk_model_wrench
from ..tools.utils import quat_error_v_ca

class MpcWrench():
    def __init__(self, n_others=0, skip_build=False):
        # Skip Acados code generation and compilation if True (rebuild needed if model or parameters change)
        self.skip_build = skip_build

        # Define the controller parameters
        self.dt = 0.2               # MPC time step [s]
        self.Nx = 30                # Prediction horizon, states             
        self.Q = np.diag([          # State weighting matrix
            1e0, 1e0, 1e0,
            1e1, 1e1, 1e1, 
            5e1, 5e1, 5e1, 
            1e1, 1e1, 1e1])           
        self.R = np.diag([          # State weighting matrix
            1e-1, 1e-1, 1e-1,
            1e1, 1e1, 1e1]) 
        self.P = 20 * self.Q        # Terminal state weighting matrix

        # Number of other agents
        self.n_others = n_others

        # Initial state (position, velocity, quaternion, angular velocity)
        self.x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # State constraints
        self.v_max = 0.5    # Max velocity [m/s]
        self.w_max = 1.0    # Max angular velocity [rad/s]
        self.idxbx = np.array([3, 4, 5, 10, 11, 12])  # velocity and angular velocity indices
        self.n_state_constraints = len(self.idxbx)

        # Slack variable weights for collision avoidance
        self.W_slack = np.array([1e5]*self.n_others)
        # Slack variable weights for state constraints
        self.W_slack_state = np.array([1e6]*self.n_state_constraints)
        self.idx_slack = np.arange(len(self.W_slack))
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
        ocp.parameter_values = np.zeros(nx+nu+self.n_others*3)

        # Get variables
        x_ref = model.p[:nx]
        u_ref = model.p[nx:nx+nu]

        # Define the cost function
        ocp.cost.cost_type = 'NONLINEAR_LS'
        ocp.cost.cost_type_e = 'NONLINEAR_LS'

        # Set the weighting matrices
        ocp.cost.W = block_diag(self.Q, self.R)
        ocp.cost.W_e = block_diag(self.P)

        q_error_v = quat_error_v_ca(model.x[6:10], x_ref[6:10])
        ocp.model.cost_y_expr = ca.vertcat(
            model.x[0:3] - x_ref[0:3],          # Position error
            model.x[3:6] - x_ref[3:6],          # Velocity error
            q_error_v,                          # Attitude error (vector part of quaternion error)
            model.x[10:13] - x_ref[10:13],      # Angular velocity error
            model.u - u_ref,                    # Control error
        )
        # Terminal cost 
        ocp.model.cost_y_expr_e = ca.vertcat(
            model.x[0:3] - x_ref[0:3],
            model.x[3:6] - x_ref[3:6],
            q_error_v,
            model.x[10:13] - x_ref[10:13],
        )
        ocp.cost.yref = np.zeros(ocp.model.cost_y_expr.shape[0])
        ocp.cost.yref_e = np.zeros(ocp.model.cost_y_expr_e.shape[0])

        # Set constraints on U
        ocp.constraints.lbu = model.u_min
        ocp.constraints.ubu = model.u_max
        ocp.constraints.idxbu = np.arange(nu)

        # Set constraints on X (velocity and angular velocity bounds)
        ocp.constraints.lbx = np.array([-self.v_max, -self.v_max, -self.v_max, 
                                         -self.w_max, -self.w_max, -self.w_max])
        ocp.constraints.ubx = np.array([+self.v_max, +self.v_max, +self.v_max, 
                                         +self.w_max, +self.w_max, +self.w_max])
        ocp.constraints.idxbx = self.idxbx

        # Terminal state constraints
        ocp.constraints.lbx_e = ocp.constraints.lbx
        ocp.constraints.ubx_e = ocp.constraints.ubx
        ocp.constraints.idxbx_e = self.idxbx

        # Soft constraints on state bounds (to avoid infeasibility)
        ocp.constraints.idxsbx = np.arange(self.n_state_constraints)
        ocp.constraints.idxsbx_e = np.arange(self.n_state_constraints)

        # Set up the constraints for collision avoidance
        ocp.constraints.lh = np.full(self.n_others, -1e9)   # lower bounds on con_h_expr
        ocp.constraints.uh = np.zeros(self.n_others)  # no upper bounds (one-sided constraint)  
        ocp.constraints.idxsh = self.idxsh  # index of slack variables corresponding to con_h_expr
        ocp.constraints.lh_e = np.full(self.n_others, -1e9)   # lower bounds on con_h_expr
        ocp.constraints.uh_e = np.zeros(self.n_others)  # no upper bounds (one-sided constraint)  
        ocp.constraints.idxsh_e = self.idxsh  # index of slack variables corresponding to con_h_expr

        # Set slack variables for collision avoidance (nonlinear constraints)
        # and state constraints (box constraints on x)
        # Combine slack weights: first for state constraints (sbx), then for nonlinear (sh)
        n_slack_total = self.n_state_constraints + self.n_others
        W_slack_combined = np.concatenate([self.W_slack_state, self.W_slack])
        zl_combined = np.zeros(n_slack_total)
        zu_combined = np.zeros(n_slack_total)

        ocp.cost.Zl = W_slack_combined
        ocp.cost.Zu = W_slack_combined
        ocp.cost.zl = zl_combined
        ocp.cost.zu = zu_combined
        ocp.cost.Zl_e = W_slack_combined
        ocp.cost.Zu_e = W_slack_combined
        ocp.cost.zl_e = zl_combined
        ocp.cost.zu_e = zu_combined

        # Set initial state
        ocp.constraints.x0 = self.x0

        # Set the prediction horizon
        ocp.solver_options.N_horizon = self.Nx
        ocp.solver_options.tf = self.dt * self.Nx

        # Set the solver options
        ocp.solver_options.nlp_solver_type = "SQP_RTI"
        ocp.solver_options.nlp_solver_max_iter = 1
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.qp_solver_cond_N = self.Nx
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.sim_method_num_stages = 4
        ocp.solver_options.sim_method_num_steps = 3
        ocp.solver_options.print_level = 0

        ocp_solver = AcadosOcpSolver(ocp, json_file=json_path, 
                                      generate=not self.skip_build,
                                      build=not self.skip_build)
        return ocp_solver

    def reset_solver(self, x0):
        """Full trajectory reset: clear solver internals first, then seed all stages with x0."""
        self.solver.reset()
        x_flat = x0.flatten().copy()
        q = x_flat[6:10]
        norm = np.linalg.norm(q)
        if norm > 1e-6:
            x_flat[6:10] = q / norm
        else:
            # fallback to identity quaternion
            x_flat[6] = 1.0
            x_flat[7:10] = 0.0
        for k in range(self.Nx + 1):
            self.solver.set(k, "x", x_flat)
        for k in range(self.Nx):
            self.solver.set(k, "u", np.zeros(self.nu))

    def _detect_discontinuity(self, x0, dt):
        """Return True if any body's position or velocity is inconsistent with kinematics."""
        a_max = 0.2  # [m/s^2]
        v_max = self.v_max  # [m/s]
        
        # compute tolerances for position and velocity
        margin = 5.0
        pos_tol = margin * (v_max * dt + a_max * dt**2/2)   # [m]
        vel_tol = margin * a_max * dt                       # [m/s]

        p, v = x0[:3], x0[3:6]
        p_prev, v_prev = self._prev_x0[:3], self._prev_x0[3:6]

        if np.linalg.norm(p - (p_prev + v_prev * dt)) > pos_tol or np.linalg.norm(v - v_prev) > vel_tol:
            return True
        return False

    def get_input(self, x0, x_ref, x_others=[]):
        # Detect sudden state discontinuities
        now = time.perf_counter()
        if hasattr(self, '_prev_x0') and hasattr(self, '_prev_call_time'):
            dt_call = now - self._prev_call_time
            if dt_call <= 0.0 or dt_call > 10.0 or self._detect_discontinuity(x0, dt_call):
                print("State discontinuity detected - resetting solver trajectory.")
                self.reset_solver(x0)
        self._prev_call_time = now
        self._prev_x0 = x0.copy()

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
        for k in range(self.Nx + 1):            
            # reference state
            if k < len_x_ref:
                p_k = x_ref[:,k]
            else:
                p_k = x_ref[:,-1]
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
            print(f"Solver failed with status {status}. Resetting.")
            self.reset_solver(x0)
            self.solver.set(0, "lbx", x0.flatten())
            self.solver.set(0, "ubx", x0.flatten())
            status = self.solver.solve()
            if status == 0:
                u_opt = self.solver.get(0, 'u')
            if status != 0:
                print(f"Still failing. Cold-starting from current state.")
                self.reset_solver(x0)
                self.solver.set(0, "lbx", x0.flatten())
                self.solver.set(0, "ubx", x0.flatten())
                status = self.solver.solve()
                if status == 0:
                    u_opt = self.solver.get(0, 'u')
                else:
                    print(f"Solver failed completely. Zeroing controls.")
                    u_opt = np.zeros(self.nu)
        
        # get solution
        x_pred = np.ndarray((self.Nx+1, self.nx))
        for i in range(self.Nx):
            x_pred[i,:] = self.solver.get(i, "x")
        x_pred[self.Nx,:] = self.solver.get(self.Nx, "x")
        
        return u_opt, x_pred