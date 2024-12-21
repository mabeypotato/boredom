classdef controllerLoop < handle
    properties
        A_C
        B_C
        C_C
        D_C
        A_F
        B_F
        C_F
        D_F
        x_C
        x_F
        limit
        tau_eq
        Ts
    end
    methods
        function self = controllerLoop(P)
            % initialized control and prefilter
            self.A_C = P.A_C;
            self.B_C = P.B_C;
            self.C_C = P.C_C;            
            self.D_C = P.D_C; 
            self.A_F = P.A_F;
            self.B_F = P.B_F;
            self.C_F = P.C_F;            
            self.D_F = P.D_F;
            self.x_C = zeros(size(P.A_C, 1), 1);
            self.x_F = zeros(size(P.A_F, 1), 1);
            self.limit = P.tau_max;
            self.tau_eq = P.tau_eq;
            self.Ts = P.Ts;
        end
        function tau = update(self, theta_r, y)
            theta = y(1);
            
            % prefilter the reference command
            % solve differential equation defining prefilter
            if length(self.x_F)==0
                theta_c_filtered = self.D_F*theta_r;
            else
                N = 10; % number of Euler integration steps for each sample
                for i=1:N
                    self.x_F = self.x_F + self.Ts/N*(...
                        self.A_F * self.x_F + self.B_F * theta_r...
                        );
                end
                % output equation for the prefilter
                theta_c_filtered = self.C_F * self.x_F + self.D_F * theta_r;
            end
            % error signal
            error = theta_c_filtered - theta;
    
            % solve differential equation defining controller
            N = 10; % number of Euler integration steps for each sample
            for i=1:N
                self.x_C = self.x_C + self.Ts/N * (...
                    self.A_C * self.x_C + self.B_C * error...
                    );
            end
            % output equation for the controller
            tau_tilde = self.C_C * self.x_C + self.D_C * error;

            % compute total torque
            %tau = self.saturate( self.tau_eq + tau_tilde);
            tau = self.tau_eq + tau_tilde;
        end
        function out = saturate(self,u)
            if abs(u) > self.limit
                u = self.limit*sign(u);
            end
            out = u;
        end        
    end
end