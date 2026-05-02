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
f_s = 3.50 : 0.50 : 6.50; % oscillation frequency for F1 cars 
z_s = 0.30 : 0.05: 0.60; % damping ratio for F1 cars   

% extract the bounds
f_s_min = min(f_s);
f_s_max = max(f_s);

z_s_min = min(z_s);
z_s_max = max(z_s);

% f = 1/(2*pi) * angular frequency to find spring constant range
k_s = f_s.^2 .* (2*pi)^2 * m_s;

% z = c/(2*sqrt(k*m)) to find damper constant range
c_s = z_s .* (2 .* sqrt(k_s .* m_s));


% to determine tire stiffness we use f_t = seperation_factor * f_s
seperation_factor = 2.5; % ensures significant optimization results  | range: 
f_t = seperation_factor .* f_s;

k_t = f_t.^2 .* (2*pi)^2 * m_u;


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

% pass spatial and temporal values into kinematics function
differential = @(t, x) kinematics(t,        ...
                                  x,        ...
                                  m_s,      ...
                                  m_u,      ...
                                  min(k_s), ...
                                  min(c_s), ...
                                  min(k_t), ...
                                  y_r);

% set ode tolerance
opts = odeset( ...
    'RelTol', 1e-9, ...
    'AbsTol', 1e-12, ...
    'MaxStep', (L3 / v) / 10 );   % force sampling of kerb


% extract computed values
[time, X] = ode45(differential, t_simframe, xnaught, opts);

y_s = X(:,1); % sprung mass displacement
v_s = X(:,2); % sprung mass velocity
y_u = X(:,3); % unsprung mass displacement
v_u = X(:,4); % unsprung mass velocity


%% Graphing
figure(Name = 'F1 Quarter-Suspension Kinematics', Color = 'black');

subplot(2,1,1)
plot(time, y_s, LineWidth = 1.5); hold on
plot(time, y_u, LineWidth = 1.5);
ylabel('Displacement [m]');
legend('Sprung', 'Unsprung');
grid on

subplot(2,1,2)
plot(time, v_s, LineWidth = 1.5); hold on
plot(time, v_u, LineWidth = 1.5);
ylabel('Velocity [m/s]');
xlabel('Time [s]');
legend('Sprung', 'Unsprung');
grid on


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

