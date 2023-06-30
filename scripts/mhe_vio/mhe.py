#!/usr/bin/env python3
import numpy as np
import casadi as ca

class MHE_ESTIMATOR:
    def __init__(self, mhe_configs):
        self.N_MHE      = mhe_configs['N']
        self.dt         = 1.0/mhe_configs['rate']
        self.Px         = mhe_configs['Px']
        self.Pp         = mhe_configs['Pp']
        self.Pvio       = mhe_configs['Pvio']
        self.Pgps       = mhe_configs['Pgps']
        self.d_gps_x    = mhe_configs['d_gps_x']

        self.n_states   = 6
        self.n_meas     = 11
        self.n_params   = 5

        self.cmd_start  = 9
        
        self.previous_sol = np.zeros(self.n_states*self.N_MHE + self.n_params)
        self.previous_sol[-5:-3] = 1

        # Set solution upper bounds
        self.ubx = ca.DM.inf((self.n_states*self.N_MHE + self.n_params, 1))
        # Set mu/nu upper bound to 1
        self.ubx[-5:-3] = 1
        # # Set roll upper bound to pi
        # self.ubx[3:self.n_states:self.n_states*self.N_MHE] = ca.pi
        # # Set pitch upper bound to pi/2
        # self.ubx[4:self.n_states:self.n_states*self.N_MHE] = ca.pi
        # # Set yaw upper bound to pi
        # self.ubx[5:self.n_states:self.n_states*self.N_MHE] = ca.pi

        self.lbx = -ca.DM.inf((self.n_states*self.N_MHE + self.n_params, 1))
        self.lbx[-5:-3] = 0
        # # Set roll upper bound to -pi
        # self.lbx[3:self.n_states:self.n_states*self.N_MHE] = -ca.pi
        # # Set pitch upper bound to -pi/2
        # self.lbx[4:self.n_states:self.n_states*self.N_MHE] = -ca.pi
        # # Set yaw upper bound to -pi
        # self.lbx[5:self.n_states:self.n_states*self.N_MHE] = -ca.pi

        self.set_x_dot()
        self.set_mhe()

    def set_x_dot(self):
        '''
        Lets define the integrator function
        '''
        # state symbolic variables
        x       = ca.SX.sym('x', 3)
        theta   = ca.SX.sym('theta', 3)
        states  = ca.vertcat(x, theta)

        # control symbolic variables
        controls = ca.SX.sym('u', 2)

        # traversability
        mu = ca.SX.sym('mu')
        nu = ca.SX.sym('nu')

        Rx = ca.vertcat(
            ca.horzcat( ca.cos(theta[2]) * ca.cos(theta[1])),
            ca.horzcat( ca.sin(theta[2]) * ca.cos(theta[1])),
            ca.horzcat(-ca.sin(theta[1])))

        Jx = Rx * controls[0] * mu

        Rtheta = ca.vertcat(
            ca.horzcat( ca.cos(theta[0]) * ca.tan(theta[1])),
            ca.horzcat(-ca.sin(theta[0])),
            ca.horzcat( ca.cos(theta[0]) / ca.cos(theta[1])))

        Jtheta = Rtheta * controls[1] * nu

        RHS = ca.vertcat(Jx, Jtheta)

        # maps controls from [v, omega].T to [x, theta].T
        self.x_dot = ca.Function('f', [states, controls, mu, nu], [RHS])

    def set_mhe(self):
        '''
        Run optmizer for traversability coefficients
        '''
        # States
        X = ca.SX.sym('x', self.n_states, self.N_MHE)
        # Input measurements + solutions from previous iteration
        P = ca.SX.sym('P', self.n_meas*self.N_MHE + self.n_states + self.n_params)
        
        # System Parameters
        mu = ca.SX.sym('mu', 1)
        nu = ca.SX.sym('nu', 1)
        x_off = ca.SX.sym('x_off', 1)
        y_off = ca.SX.sym('y_off', 1)
        theta_off = ca.SX.sym('theta_off', 1)

        sys_p = ca.vertcat(mu, nu, x_off, y_off, theta_off)

        prev_x = P[self.n_meas*self.N_MHE:self.n_meas*self.N_MHE+self.n_states]
        prev_p = P[self.n_meas*self.N_MHE+self.n_states:]
        meas = ca.reshape(P[:self.n_meas*self.N_MHE], self.n_meas, self.N_MHE)

        '''
        Create cost function
        '''
        f = 0
        # Cost for previous prediction
        f += self.Px[0,0] * (prev_x[0] - X[0,0])**2
        f += self.Px[1,1] * (prev_x[1] - X[1,0])**2
        f += self.Px[2,2] * (prev_x[2] - X[2,0])**2
        # f += self.Px[3,3] * (ca.mod(prev_x[3] - X[3,0] + ca.pi, 2*ca.pi) - ca.pi)**2
        # f += self.Px[4,4] * (ca.mod(prev_x[4] - X[4,0] + ca.pi, 2*ca.pi) - ca.pi)**2
        # f += self.Px[5,5] * (ca.mod(prev_x[5] - X[5,0] + ca.pi, 2*ca.pi) - ca.pi)**2
        f += self.Px[3,3] * (1 - ca.cos(prev_x[3] - X[3,0]))**2
        f += self.Px[4,4] * (1 - ca.cos(prev_x[4] - X[4,0]))**2
        f += self.Px[5,5] * (1 - ca.cos(prev_x[5] - X[5,0]))**2

        # Cost for previous parameters estimation
        f += (prev_p - sys_p).T @ self.Pp @ (prev_p - sys_p)

        # Cost to maximize traversability
        #f += (1-sys_p[:2]).T @ self.Ptrav @ (1-sys_p[:2])
                
        for i in range(self.N_MHE):
            st = X[:, i]
            sens = meas[:self.cmd_start, i]

            # VIO
            # f += self.Pvio[0,0] * (st[0] - (sens[0]*ca.cos(theta_off) - sens[1]*ca.sin(theta_off) + x_off))**2
            # f += self.Pvio[1,1] * (st[1] - (sens[0]*ca.sin(theta_off) + sens[1]*ca.cos(theta_off) + y_off))**2
            # f += self.Pvio[2,2] * (st[2] - sens[2])**2
            f += self.Pvio[0,0] * (st[0] - sens[0])**2
            f += self.Pvio[1,1] * (st[1] - sens[1])**2
            f += self.Pvio[2,2] * (st[2] - sens[2])**2
            #f += self.Pvio[3,3] * (ca.mod(st[3] - sens[3] + ca.pi, 2*ca.pi) - ca.pi)**2
            #f += self.Pvio[4,4] * (ca.mod(st[4] - sens[4] + ca.pi, 2*ca.pi) - ca.pi)**2
            #f += self.Pvio[5,5] * (ca.mod(st[5] - (sens[5] + theta_off) + ca.pi, 2*ca.pi) - ca.pi)**2
            f += self.Pvio[3,3] * (1 - ca.cos(st[3] - sens[3]))**2
            f += self.Pvio[4,4] * (1 - ca.cos(st[4] - sens[4]))**2
            # f += self.Pvio[5,5] * (1 - ca.cos(st[5] - (sens[5] + theta_off)))**2
            f += self.Pvio[5,5] * (1 - ca.cos(st[5] - sens[5]))**2

            # GPS
            trans_x = sens[0] + ca.cos(sens[5])*self.d_gps_x
            trans_y = sens[1] + ca.sin(sens[5])*self.d_gps_x
            f += sens[8] * self.Pgps[0,0] * (trans_x*ca.cos(theta_off) - trans_y*ca.sin(theta_off) + x_off - sens[6])**2
            f += sens[8] * self.Pgps[1,1] * (trans_x*ca.sin(theta_off) + trans_y*ca.cos(theta_off) + y_off - sens[7])**2
            # f += sens[8] * self.Pgps[0,0] * (sens[0] + x_off + ca.cos(sens[5] + theta_off)*self.d_gps_x - sens[6])**2
            # f += sens[8] * self.Pgps[1,1] * (sens[1] + y_off + ca.sin(sens[5] + theta_off)*self.d_gps_x - sens[7])**2

        '''
        Now, let's define the constraints
        ''' 
        g = []
        for i in range(self.N_MHE-1):
            st = X[:, i]
            st_next = X[:, i+1]
            con = meas[self.cmd_start:, i]

            # Runge-Kutta integration
            k1 = self.x_dot(st, con, mu, nu)
            k2 = self.x_dot(st + self.dt/2*k1, con, mu, nu)
            k3 = self.x_dot(st + self.dt/2*k2, con, mu, nu)
            k4 = self.x_dot(st + self.dt*k3, con, mu, nu)

            st_next_RK4 = st + (self.dt / 6) * (k1 + 2*k2 + 2*k3 + k4)

            g = ca.vertcat(g, st_next - st_next_RK4)
        #     g = ca.vertcat(g, ca.DM.zeros(6))

        # for i in range(self.N_MHE-1):
        #     st = X[:, i]
        #     st_next = X[:, i+1]
        #     con = meas[self.cmd_start:, i]

        #     # Runge-Kutta integration
        #     k1 = self.x_dot(st, con, mu, nu)
        #     k2 = self.x_dot(st + self.dt/2*k1, con, mu, nu)
        #     k3 = self.x_dot(st + self.dt/2*k2, con, mu, nu)
        #     k4 = self.x_dot(st + self.dt*k3, con, mu, nu)

        #     st_next_RK4 = st + (self.dt / 6) * (k1 + 2*k2 + 2*k3 + k4)

        #     f += (st_next - st_next_RK4).T @ self.Pv @ (st_next - st_next_RK4)

        OPT_variables = ca.vertcat(X.reshape((-1, 1)), mu, nu, x_off, y_off, theta_off)

        nlp = {}        # NLP declaration
        nlp['x'] = OPT_variables    # decision vars
        nlp['f'] = f    # objective
        nlp['g'] = g    # constraints
        nlp['p'] = P    # parameters

        # Define solver options
        # opts = {
        #     'ipopt.print_level':0,
        #     'print_time':0,
        #     'ipopt.acceptable_tol':0.005,
        #     'ipopt.tol':0.005,
        #     'ipopt.max_cpu_time':0.5}

        opts = {'ipopt': {'max_iter': 100,
                          'print_level': 0,
                          'max_cpu_time': 0.5},
                'print_time': 0,
                'jit': True,
                'compiler': 'shell',
                'jit_options': {'compiler': 'gcc', 'flags': ['-O3']}}
        
        # Create solver instance
        self.solver = ca.nlpsol('F', 'ipopt', nlp, opts)

    def run_mhe(self, horizon_data):
        p = []
        for i in range(self.N_MHE):
            p.extend([horizon_data['odom']['data'][i][0],
                      horizon_data['odom']['data'][i][1],
                      horizon_data['odom']['data'][i][2],
                      horizon_data['odom']['data'][i][3],
                      horizon_data['odom']['data'][i][4],
                      horizon_data['odom']['data'][i][5],
                      horizon_data['gnss']['data'][i][0],
                      horizon_data['gnss']['data'][i][1],
                      horizon_data['gnss']['available'][i],
                      horizon_data['cmd']['data'][i][0],
                      horizon_data['cmd']['data'][i][1]])

        temp = ca.reshape(self.previous_sol[:self.n_states*self.N_MHE], self.n_states, self.N_MHE)

        p.extend([temp[0,1],                # previous x
                  temp[1,1],                # previous y
                  temp[2,1],                # previous z
                  ca.mod(temp[3,1] + ca.pi, 2*ca.pi) - ca.pi,   # previous roll
                  ca.mod(temp[4,1] + ca.pi, 2*ca.pi) - ca.pi,   # previous pitch
                  ca.mod(temp[5,1] + ca.pi, 2*ca.pi) - ca.pi,   # previous yaw
                #   temp[3,1],                # previous roll
                #   temp[4,1],                # previous pitch
                #   temp[5,1],                # previous yaw
                  self.previous_sol[-5],    # previous mu
                  self.previous_sol[-4],    # previous nu
                  self.previous_sol[-3],    # previous x_off
                  self.previous_sol[-2],    # previous y_off
                #   self.previous_sol[-1]])   # previous theta_off
                  ca.mod(self.previous_sol[-1] + ca.pi, 2*ca.pi) - ca.pi])   # previous theta_off

        p = ca.DM(p)

        # Solve the problem using a guess
        sol = self.solver(x0=self.previous_sol,
                          ubx=self.ubx,
                          lbx=self.lbx,
                          ubg=ca.DM.zeros((self.n_states*(self.N_MHE-1), 1)),
                          lbg=ca.DM.zeros((self.n_states*(self.N_MHE-1), 1)),
                          p=p)

        self.previous_sol = np.array(sol['x']).astype(float)
        
        # Return solution x, y, theta, mu, nu, d_heading
        sol_x = sol['x'][:self.n_states*self.N_MHE]
        sol_reshaped = ca.reshape(sol_x, self.n_states, self.N_MHE)
        pred_states = np.array(sol_reshaped[:,-1]).astype(float)

        mu          = float(sol['x'][-5])
        nu          = float(sol['x'][-4])
        x_off       = float(sol['x'][-3])
        y_off       = float(sol['x'][-2])
        theta_off   = float(sol['x'][-1])

        return pred_states, mu, nu, x_off, y_off, theta_off