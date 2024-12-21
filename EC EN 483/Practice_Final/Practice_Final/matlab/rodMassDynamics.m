classdef rodMassDynamics < handle
    properties
        state
        m
        ell
        taubar
        k1
        k2
        b
        g
        Ts
        torque_limit
    end
    methods
        %---constructor-------------------------
        function self = rodMassDynamics(alpha, P)
            % Initial state conditions
            self.state = [...
                        0;...  % initial angle
                        0;...  % initial angular rate
                        ]; 
            self.g = 9.8;
            self.ell = 0.25 * (1+2*alpha*rand-alpha);
            self.m = 0.1 * (1+2*alpha*rand-alpha);
            self.taubar = 3;
            self.k1 = 0.02 * (1+2*alpha*rand-alpha);
            self.k2 = 0.01 * (1+2*alpha*rand-alpha);
            self.b  = 0.1 * (1+2*alpha*rand-alpha);  
            self.Ts = 0.01;
        end
        function y = update(self, u)
            % saturate the input
            u = self.saturate(u, self.torque_limit);
            self.rk4_step(u);
            y = self.h();
        end
        function xdot = f(self, state, u)
            % Return xdot = f(x,u)
            theta = state(1);
            thetadot = state(2);
            tau = self.saturate(u, self.taubar);
            
            % The equations of motion.
            thetaddot = (-self.g/self.ell)*cos(theta)...
                + (-self.k1/(self.m*self.ell^2))*theta...
                + (-self.k2/(self.m*self.ell^2))*(theta^3)...
                - self.b/(self.m*self.ell^2)*thetadot...
                + (1/(self.m*self.ell^2))*tau; 
            xdot = [thetadot; thetaddot];
        end
        function y = h(self)
            % Return y = h(x)
            theta = self.state(1);
            y = theta;
        end
        function self = rk4_step(self, u)
            % Integrate ODE using Runge-Kutta RK4 algorithm
            F1 = self.f(self.state, u);
            F2 = self.f(self.state + self.Ts/2*F1, u);
            F3 = self.f(self.state + self.Ts/2*F2, u);
            F4 = self.f(self.state + self.Ts*F3, u);
            self.state = self.state...
                + self.Ts/6 * (F1 + 2*F2 + 2*F3 + F4);
        end
        function out = saturate(self, in, limit)
            if abs(in) > limit
                out = limit * sign(in);
            else 
                out = in;
            end
        end
    end
end


