%% F1 Suspension Model

% Units
% distance = m
% mass = kg
% frequency = Hz
% time = s

%% Base Parameters

% define sprung mass and unsprung mass from total mass
m = 800; % total F1 car mass
q_m = m ./ 4; % approximate quarter suspension mass

m_s = q_m .* .85; % sprung mass | range .85 - .90
m_u = q_m - m_s; % unsprung mass

% define velocity
v = 200; % approximate speed at contact km/hr
v = v / 60 / 60 * 1000; % translate km/hr to m/s

% establish base parameters
base.m_s = m_s;
base.m_u = m_u;
base.v = v;
base.delta_limit_s = 0.005; % spring deflection limit
base.delta_limit_t = 0.004; % tire deflection limit

%% Independent Variables
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


%% Kerb Geometry & Timeframe

% model the kerb geometry and base excitation
h = .04; % define the kerb height
L1 = .25; L2 = .75; L3 = 1; % split the kerb segment inclination along three points

% encapsulate into base parameters
base.h = h;
base.L1 = L1;
base.L2 = L2;
base.L3 = L3;

% derive the time for the simulation
t_horizon = base.L3/base.v; % time horizon at final kerb length

p_s = 1/f_s_min; % maximum period of oscilation for sprung mass

discernment_factor = 4; % approximate factor of time beyond the oscilation period
t_settle = p_s * discernment_factor; % time needed for oscilation to settle

t_sim = t_horizon + t_settle; % actual simulation runtime
t_common = linspace(0, t_sim, 1000); % time variation for consistent spatial variance


% encapsulate the translation from spatial to temporal height into base
base.y_r = @(t, p) ...
            (p.v*t < p.L1)               .* (p.h/p.L1 .* p.v*t) + ...                           % incline
            (p.L1 <= p.v*t & p.v*t < p.L2) .* (p.h)             + ...                           % flat 
            (p.L2 <= p.v*t & p.v*t < p.L3) .* (p.h .* (1 - (p.v*t - p.L2) ./ (p.L3 - p.L2)));   % decline


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

% create cell to host varying coefficients
results = cell(length(k_s_range), length(c_s_range));


% initiate for loop dependent on spring and damper coefficient variations
for i = 1:numel(k_s_range)
    for j = 1:numel(c_s_range)

        % simplify base
        p = base;
        % set coefficient values
        p.k_s = k_s_range(i);
        p.c_s = c_s_range(j);
        p.k_t = k_t_range(i); % tire spring depends on sprung mass spring

        % compute and extract values
        [t, X] = ode45( @(t, x) ...
            kinematics(t, x, p), ... % use anonymous function for temporal and spatial sampling
                       t_common, ...
                        xnaught, ...
                 opts);

        % store computed values
        results{i,j}.p = p;
        results{i,j}.x = X;
        results{i,j}.t = t;

        % store coefficients
        results{i,j}.k_s = p.k_s;
        results{i,j}.c_s = p.c_s;
        results{i,j}.k_t = p.k_t;

        % compute acceleration and suspension deformation
        y_s = X(:,1); v_s = X(:,2); % sprung mass kinematics
        y_u = X(:,3); v_u = X(:,4); % unsprung mass kinematics

        % store the acceleration
        results{i,j}.a_s = (- p.k_s .* (y_s - y_u) - p.c_s .* (v_s - v_u)) / p.m_s;
        results{i,j}.a_u = (p.k_s .* (y_s - y_u) + p.c_s .* (v_s - v_u) - p.k_t * ( y_u - p.y_r(t, p))) / p.m_u;

        % store the deflection
        results{i,j}.delta_s = y_s - y_u;
        results{i,j}.delta_t = y_u - p.y_r(t, p);

        
    end
end


%% Plotting
% prepare color sort
colors = ['g', 'c', 'm', 'y'];

% extract values for plotting
for j = 1:length(c_s_range) % separate damping ratios by figures
    fig(j) = figure(Name = sprintf('Set (%d)', j), Color = 'black');

    % initiate tiles
    tiledlayout(3, 1, TileSpacing = 'compact', Padding = 'compact')
    sgtitle(sprintf('Damping Coefficient: c = %.2e kg/s', c_s_range(j)))

    ax(1) = nexttile([1, 1]); hold on; grid on
    ylabel('Suspension Deflection [m]');

    ax(2) = nexttile([1, 1]); hold on; grid on
    ylabel('Tire Deflection [m]');

    ax(3) = nexttile([1, 1]); hold on; grid on
    ylabel('Acceleration [m/s^2]');
    xlabel('Time [s]');

    for i = 1:length(k_s_range) % iterate through spring coefficients
        % access the corresponding data
        R = results{i,j};

        % plot spring deflection
        plot(ax(1), R.t, R.delta_s, LineWidth = 1.5, Color = colors(i), ...
             DisplayName = sprintf('k = %.1f kN/m', (R.k_s / 1000)));

        % plot tire deflection
        plot(ax(2), R.t, R.delta_t, LineWidth = 1.5, Color = colors(i), ...
             DisplayName = sprintf('k = %.1f kN/m', (R.k_s / 1000)));

        % plot the acceleration
        plot(ax(3), R.t, R.a_s, LineWidth = 1.5, Color = colors(i), ...
             DisplayName = sprintf('k = %.1f kN/m', (R.k_s / 1000)));
        plot(ax(3), R.t, R.a_u, LineWidth = 1.5, Color = colors(i), ...
             LineStyle = '--', HandleVisibility = 'off');

    end

    % plot deformation threshold
    yline(ax(1), R.p.delta_limit_s, Color = 'r', LineWidth = 1.5, ...
         LineStyle = '--', DisplayName = sprintf('\\delta limit'));

    yline(ax(1), -R.p.delta_limit_s, Color = 'r', LineWidth = 1.5, ...
         LineStyle = '--', HandleVisibility = 'off');
    
   
    yline(ax(2), R.p.delta_limit_t, Color = 'r', LineWidth = 1.5, ...
         LineStyle = '--', DisplayName = sprintf('\\delta limit'));

    yline(ax(2), -R.p.delta_limit_t, Color = 'r', LineWidth = 1.5, ...
         LineStyle = '--', HandleVisibility = 'off');

    % set x limit to show significant values
    xlim(ax(1), [0, 1.0]);
    xlim(ax(3), [0, 0.1]);

    % show plot legends
    legend(ax(1), 'show');
    legend(ax(2), 'show');
    legend(ax(3), 'show');

    % create style legend
    % dummy lines
    dummy_sprung = plot(nan, nan, '-',  'LineWidth', 1.6, Color = 'w');
    dummy_unsprung = plot(nan, nan, '--', 'LineWidth', 1.6, Color = 'w');

    lgd_style = legend([dummy_sprung, dummy_unsprung], {'Sprung', 'Unsprung'});
    lgd_style.Layout.Tile = 'south';

end

%% Functions

% solve for velocity and acceleration at each point in time
function derivatives = kinematics(t, x, p)
    
    % unpack the state
    y_s = x(1); % sprung mass displacement
    v_s = x(2); % sprung mass velocity
    y_u = x(3); % unsprung mass displacement
    v_u = x(4); % unsprung mass velocity

    % calculate the accelerations for integration
    a_s = (- p.k_s .* (y_s - y_u) - p.c_s .* (v_s - v_u)) / p.m_s;
    a_u = (p.k_s .* (y_s - y_u) + p.c_s .* (v_s - v_u) - p.k_t * ( y_u - p.y_r(t, p))) / p.m_u;

    % return time derivatives of the state (will not be stored)
    derivatives = [v_s; % sprung mass velocity
                   a_s; % sprung mass acceleration
                   v_u; % unsprung mass velocity
                   a_u]; % unsprung mass acceleration
end
