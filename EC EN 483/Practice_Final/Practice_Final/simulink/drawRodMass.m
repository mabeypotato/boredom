function drawRodMass(u,L,w,R,H)

    % process inputs to function
    theta    = u(1);
    t        = u(2);
    
    % define persistent variables 
    persistent rod_handle
    persistent mass_handle
    
    % first time function is called, initialize plot and persistent vars
    if t==0,
        figure(1), clf
        wall_X = [-H/3,0,0,-H/3];
        wall_Y = [H, H, -H, -H];
        fill(wall_X,wall_Y,'k'); % draw wall
        hold on
        plot([0,L],[0,0],'--k'); % draw zero line
        rod_handle  = drawRod(theta, L, w, []);
        mass_handle = drawMass(theta, L, R, []);
        axis([-2*L, 2*L, -2*L, 2*L]);
    
        
    % at every other time step, redraw base and rod
    else 
        drawRod(theta, L, w, rod_handle);
        drawMass(theta, L, R, mass_handle);
    end
end

   
%
%=======================================================================
% drawRod
% draw the rod
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawRod(theta, L, w, handle)
  
  pts = [...
      0, -w/2;...
      L, -w/2;...
      L, w/2;...
      0, w/2;...
      ]';
  pts = [cos(theta), -sin(theta); sin(theta), cos(theta)]*pts;
  X = pts(1,:);
  Y = pts(2,:);
  
  if isempty(handle),
    handle = fill(X,Y,'b');
    %handle = plot(X,Y,'b');
  else
    set(handle,'XData',X,'YData',Y);
    drawnow
  end
end

%
%=======================================================================
% drawMass
% draw the mass
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawMass(theta, L, R, handle)  
  N = 10;
  th = [0:2*pi/N:2*pi];
  pts = R*[...
      cos(th);...
      sin(th);...
      ]+repmat([L;0],1,length(th));
  pts = [cos(theta), -sin(theta); sin(theta), cos(theta)]*pts;
  X = pts(1,:);
  Y = pts(2,:);
  
  if isempty(handle),
    handle = fill(X,Y,'b');
    %handle = plot(X,Y,'b');
  else
    set(handle,'XData',X,'YData',Y);
    drawnow
  end
end
 