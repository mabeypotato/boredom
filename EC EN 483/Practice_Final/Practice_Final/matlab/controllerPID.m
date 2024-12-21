classdef controllerPID < handle
    properties
        kp
        ki
        kd
        limit
        beta
        Ts
        theta_d1;
        theta_dot;
        error_d1
        integrator
        tau_eq
    end
    methods
        %----------------------------
        function self = controllerPID(P)
            self.kp = P.kp;
            self.ki = P.ki;
            self.kd = P.kd;
            self.limit = P.tau_max;
            self.beta = (2*P.sigma - P.Ts)/(2*P.sigma + P.Ts);
            self.Ts = P.Ts;
            self.theta_d1 = 0;
            self.theta_dot = 0;
            self.error_d1 = 0;
            self.integrator = 0;
            self.tau_eq = P.tau_eq;
        end
        function tau = update(self, theta_r, y)
        end
    end
end