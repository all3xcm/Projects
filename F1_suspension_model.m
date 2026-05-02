%% F1 Suspension Model

% Units
% distance = m
% mass = kg
% frequency = Hz
% time = s

%% Coefficients & Assumptions
% ==================================================
%  explore more variation of values towards the end
% ==================================================

% format the outputs
format shortE

% define sprung mass and unsprung mass from total mass
m = 800; % total F1 car mass
q_m = m ./ 4; % approximate quarter suspension mass

m_s_percentage = .85 : .01 : .90; % sprung mass percentage range
m_u_percentage = 1 - m_s_percentage; % unsprung mass percentage range

m_s = q_m .* .85; % sprung mass
m_u = q_m - m_s; % unsprung mass


% define range for spring and damper coefficients
f_s = 3.50 : 1.00 : 6.50; % oscillation frequency for F1 cars 
z_s = 0.30 : 0.10 : 0.60; % damping ratio for F1 cars   

% extract the bounds
f_s_min = min(f_s);
f_s_max = max(f_s);

z_s_min = min(z_s);
z_s_max = max(z_s);

% f = 1/(2*pi) * angular frequency to find spring constant range
k_s_range = f_s.^2 .* (2*pi)^2 * m_s;

% z = c/(2*sqrt(k*m)) to find damper constant range
c_s_range = z_s .* (2 .* sqrt(k_s_range .* m_s));


% to determine tire stiffness we use f_t = seperation_factor * f_s
seperation_factor = 2.5; % ensures significant optimization results  | range: 
f_t = seperation_factor .* f_s;

k_t_range = f_t.^2 .* (2*pi)^2 * m_u;


% define velocity
v = 200; % approximate speed at contact km/hr

v = v / 60 / 60 * 1000; % translate km/hr to m/s

%% Kerb Geometry & Timeframe

% model the kerb geometry and base excitation
h = 1; % define the kerb height
L1 = .25; L2 = .75; L3 = 1; % split the kerb segment inclination along three points


% derive the time for the simulation
t_horizon = L3/v; % time horizon at final kerb length

p_s = 1/f_s_min; % maximum period of oscilation for sprung mass

discernment_factor = 4; % approximate factor of time beyond the oscilation period
t_settle = p_s * discernment_factor; % time needed for oscilation to settle

t_sim = t_horizon + t_settle; % actual simulation runtime
t_simframe = [0, t_sim]; % simulation timeframe
t_common = linspace(0, t_sim, 1000); % time variation for consistent spatial variance


% encapsulate the spatial height
x_r = @(x) ...
         (x < L1)               .* (h/L1 .* x) + ...                     % incline
         (L1 <= x & x < L2)     .* (h)         + ...                     % flat 
         (L2 <= x & x < L3)     .* (h .* (1 - (x - L2) ./ (L3 - L2)));   % decline

% translate spatial to temporal height
y_r = @(t) x_r(v*t);


%% ODE & Kinematics

% define variable that will hold the velocities and displacements
xnaught = [0;  % y_s
           0;  % v_s
           0;  % y_u
           0]; % v_u


% set ode tolerance
opts = odeset( ...
    'RelTol', 1e-9, ...
    'AbsTol', 1e-12, ...
    'MaxStep', (L3 / v) / 10 );   % force sampling of kerb

% create cells to host varying coefficients
x_host = cell(length(k_s_range), length(c_s_range));
t_host = cell(size(x_host));

% initiate for loop dependent on spring and damper coefficient variations
for i = 1:numel(k_s_range)
    for j = 1:numel(c_s_range)

        % set coefficient values
        k_s = k_s_range(i);
        c_s = c_s_range(j);
        k_t = k_t_range(i); % tire spring depends on sprung mass spring

        % compute and extract values
        [time, X] = ode45( @(t, x) ...
            kinematics( ... % use anonymous function for temporal and spatial sampling
                       t,    ...
                       x,    ...
                       m_s,  ...
                       m_u,  ...
                       k_s,  ...
                       c_s,  ...
                       k_t,  ...
                       y_r), ...
                             ... % include remaining ode45 parameters
                 t_common,   ...
                 xnaught,    ...
                 opts);

        % store computed values
        x_host{i,j} = X;
        t_host{i,j} = time;
    end
end


%% Plotting
% prepare color sort
colors = ['r', 'c', 'm', 'y'];

% extract values for plotting
for j = 1:length(c_s_range) % separate damping ratios by figures
    fig(j) = figure(Name = sprintf('Set (%d)', j), Color = 'black');
    sgtitle(sprintf('Damping Coefficient: \\zeta = %.2e kg/s', c_s_range(j)))

    % initiate subplots
    subplot(2,1,1); hold on; grid on
    ylabel('Displacement [m]');

    subplot(2,1,2); hold on; grid on
    ylabel('Velocity [m/s]');
    xlabel('Time [s]');

    % create a list for legends
    labels = {};

    for i = 1:length(k_s_range) % iterate through spring coefficients
        % access the corresponding data
        X = x_host{i,j}; 
        %time = t_host(i,j);
        time = linspace(0, t_sim, 1000); % define a common time

        y_s = X(:,1); % sprung mass displacement
        v_s = X(:,2); % sprung mass velocity
        y_u = X(:,3); % unsprung mass displacement
        v_u = X(:,4); % unsprung mass velocity

        subplot(2,1,1)
        plot(time, y_s, LineWidth = 1.5, Color = colors(i));
        plot(time, y_u, LineWidth = 1.5, Color = colors(i), LineStyle = '--', ...
             HandleVisibility = 'off');

        subplot(2,1,2)
        plot(time, v_s, LineWidth = 1.5, Color = colors(i));
        plot(time, v_u, LineWidth = 1.5, Color = colors(i), LineStyle = '--', ...
             HandleVisibility = 'off');

        labels{end + 1} = sprintf('k = %.1f kN/m', (k_s_range(i) / 1000));

    end

    % create color legends
    subplot(2,1,1)
    
    % dummy lines
    plot(nan, nan, '-',  'LineWidth', 1.6, Color = 'w');
    plot(nan, nan, '--', 'LineWidth', 1.6, Color = 'w');
    
    labels{end + 1} = 'Sprung';
    labels{end + 1} = 'Unsprung';

    legend(labels)

    subplot(2,1,2)

    % dummy lines
    plot(nan, nan, '-',  'LineWidth', 1.6, Color = 'w');
    plot(nan, nan, '--', 'LineWidth', 1.6, Color = 'w');
    
    labels{end + 1} = 'Sprung';
    labels{end + 1} = 'Unsprung';
    
    legend(labels)

end

%% Functions

% solve for velocity and acceleration at each point in time
function derivatives = kinematics(t, xnaught, m_s, m_u, k_s, c_s, k_t, y_r)
    
    % unpack the state
    y_s = xnaught(1); % sprung mass displacement
    v_s = xnaught(2); % sprung mass velocity
    y_u = xnaught(3); % unsprung mass displacement
    v_u = xnaught(4); % unsprung mass velocity

    % calculate the accelerations for integration
    a_s = (- k_s .* (y_s - y_u) - c_s .* (v_s - v_u)) / m_s;
    a_u = (k_s .* (y_s - y_u) + c_s .* (v_s - v_u) - k_t * ( y_u - y_r(t))) / m_u;

    % return time derivatives of the state (will not be stored)
    derivatives = [v_s; % sprung mass velocity
                   a_s; % sprung mass acceleration
                   v_u; % unsprung mass velocity
                   a_u]; % unsprung mass acceleration
end

