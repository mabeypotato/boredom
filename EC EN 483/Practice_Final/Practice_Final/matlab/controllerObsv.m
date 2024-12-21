classdef controllerObsv < handle
    properties
        obsv_state
        tau_d1
        integrator
        error_d1
        K
        ki
        L
        A
        B
        C
        limit
        Ts
        tau_eq
    end
    methods
        %--------constructor--------------------
        function self = controllerObsv(P)
            self.obsv_state = [0.0; 0.0; 0.0];
            self.tau_d1 = 0.0;
            self.integrator = 0.0;
            self.error_d1 = 0.0;
            self.K = P.K;
            self.ki = P.ki2;
            self.A = P.A2;
            self.B  = P.B2;
            self.C  = P.C2;
            self.L  = P.L2;
            self.limit = P.tau_max;
            self.Ts = P.Ts;
            self.tau_eq = P.tau_eq;
        end
        function [tau, x_hat, d_hat] = update(self, theta_r, y)
        end
        function [xhat, dhat] = updateObserver(self, y)
            % update observer using RK4 integration
            F1 = self.obsv_f(self.obsv_state, y);
            F2 = self.obsv_f(self.obsv_state + self.Ts/2*F1, y);
            F3 = self.obsv_f(self.obsv_state + self.Ts/2*F2, y);
            F4 = self.obsv_f(self.obsv_state + self.Ts*F3, y);
            self.obsv_state = self.obsv_state ...
                + self.Ts/6 * (F1 + 2*F2 + 2*F3 + F4);
            xhat = self.obsv_state(1:2);
            dhat = self.obsv_state(3);
        end
        function x_hat_dot = obsv_f(self, obsv_state, y)
        end
        function self = integrateError(self, error)
        end
        function out = saturate(self,u)
            if abs(u) > self.limit
                u = self.limit*sign(u);
            end
            out = u;
        end        
    end
end