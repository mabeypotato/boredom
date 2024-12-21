rodMassParam;  % load parameters

% instantiate rodMass, controller, and reference input classes 
alpha = 0.0;
rodMass = rodMassDynamics(alpha, P);  
controller = controllerObsv(P);  
reference = signalGenerator(20*pi/180, 0.1);  
disturbance = signalGenerator(0.5, 0.0);
noise = 0.0; %signalGenerator(0.01);

% instantiate the data plots and animation
dataPlot = dataPlotter(P);
animation = rodMassAnimation(P);
dataPlotObserver = dataPlotterObserver(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
y = rodMass.h();
while t < P.t_end  
    t_next_plot = t + P.t_plot;
    while t < t_next_plot 
        r = reference.square(t);
        d = disturbance.step(t);
        n = 0.0; %noise.random(t);  % noise
        [u, xhat, dhat] = controller.update(r, y + n);  
        y = rodMass.update(u + d);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.update(rodMass.state);
    dataPlot.update(t, r, rodMass.state, u);
    dataPlotObserver.update(t, rodMass.state, xhat, d, dhat);
end

