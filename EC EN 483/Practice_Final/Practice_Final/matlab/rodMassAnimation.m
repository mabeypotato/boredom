classdef rodMassAnimation < handle
    properties
        rod_handle
        mass_handle
        L
        w
        R
        H
    end
    methods
        %------constructor-----------
        function self = rodMassAnimation(P)
            self.L = 1.0;
            self.w = 0.01;
            self.R = 0.1;
            self.H = 1.0;
            
            figure(1), clf
            wall_X = [-self.H/3,0,0,-self.H/3];
            wall_Y = [self.H, self.H, -self.H, -self.H];
            fill(wall_X,wall_Y,'k'); % draw wall
            hold on
            plot([0,self.L],[0,0],'--k'); % draw zero line
            self.drawRod(0.0);
            self.drawMass(0.0);
            axis([-2*self.L, 2*self.L, -2*self.L, 2*self.L]);
        end
        function self = update(self, x)
            theta = x(1);
            self.drawRod(theta);
            self.drawMass(theta);
            drawnow;
        end
        function drawRod(self, theta)
            pts = [...
                    0, -self.w/2;...
                    self.L, -self.w/2;...
                    self.L, self.w/2;...
                    0, self.w/2;...
                ]';
            pts = [cos(theta), -sin(theta); sin(theta), cos(theta)]*pts;
            X = pts(1,:);
            Y = pts(2,:);

            if isempty(self.rod_handle)
                self.rod_handle = fill(X, Y, 'b');
            else
                set(self.rod_handle, 'XData', X, 'YData', Y);
            end
        end
        function drawMass(self, theta)
            N = 10;
            th = [0:2*pi/N:2*pi];
            pts = self.R*[cos(th); sin(th)]...
                +repmat([self.L;0],1,length(th));
            pts = [cos(theta), -sin(theta); sin(theta), cos(theta)]*pts;
            X = pts(1,:);
            Y = pts(2,:);
            if isempty(self.mass_handle)
                self.mass_handle = fill(X,Y,'b');
            else
                set(self.mass_handle,'XData',X,'YData',Y);
            end
        end
    end
end