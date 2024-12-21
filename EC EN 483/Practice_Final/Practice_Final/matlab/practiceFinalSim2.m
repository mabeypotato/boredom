rodMassParam;  % load parameters

% instantiate rodMass, and reference input classes 
alpha = 0.0;
rodMass = rodMassDynamics(alpha, P);  

% instantiate the data plots and animation
dataPlot = dataPlotter(P);
animation = rodMassAnimation();

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % Propagate dynamics at rate Ts
    t_next_plot = t + P.t_plot;
    while t < t_next_plot 
        u = P.tau_eq;  
        y = rodMass.update(u);  
        t = t + P.Ts; 
    end
    % update animation and data plots at rate t_plot
    animation.update(rodMass.state);
    dataPlot.update(t, 0, rodMass.state, u);
end
