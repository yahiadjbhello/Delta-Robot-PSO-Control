% =========================================================================
%  run_simulation.m  —  Main Entry-Point Script
%  Delta Robot: PSO-Tuned Backstepping & Sliding Mode Controllers
% =========================================================================
%
%  DESCRIPTION:
%    This script configures the simulation environment, generates the
%    reference trajectory for the 3-DOF Delta Parallel Robot, and runs
%    the desired Simulink/Simscape co-simulation model.
%
%  USAGE:
%    1. Open MATLAB and set the working directory to the repo root.
%    2. Run this script:   >> run_simulation
%    3. Choose which model to simulate by setting MODEL_CHOICE below.
%
%  MODELS AVAILABLE:
%    'backstepping'  — PSO-tuned Backstepping controller
%    'pdsmc'         — PSO-tuned PD + Sliding Mode controller
%    'pdsmc_basic'   — PD + SMC (without PSO tuning)
%    'smc_linear'    — SMC based on linearization
%
%  REQUIREMENTS:
%    - MATLAB R2023b or later
%    - Simulink
%    - Simscape Multibody
%
% =========================================================================

clc; clear; close all;

%% -------------------------------------------------------------------------
%  USER SETTINGS  —  Edit these to configure your simulation
%  -------------------------------------------------------------------------

% Choose controller model to run:
MODEL_CHOICE = 'backstepping';  % 'backstepping' | 'pdsmc' | 'pdsmc_basic' | 'smc_linear'

% Trajectory type:
TRAJECTORY = 'circle';          % 'circle' | 'square'

% Trajectory parameters:
RADIUS     = 0.2;               % [m]  — trajectory circle radius
OMEGA      = 2;                 % [rad/s] — angular velocity
Z_OFFSET   = -0.3;              % [m]  — end-effector z height
Z_LIFT     = 0.01;              % [m/s] — slow vertical rise rate
T_END      = 12;                % [s]  — total simulation time
DT         = 0.05;              % [s]  — time step

%% -------------------------------------------------------------------------
%  ADD PATHS
%  -------------------------------------------------------------------------
addpath(fullfile(pwd, 'kinematics'));
addpath(fullfile(pwd, 'dynamics'));
addpath(fullfile(pwd, 'trajectory'));
addpath(fullfile(pwd, 'simscape'));
addpath(fullfile(pwd, 'scripts'));
addpath(fullfile(pwd, 'models'));

%% -------------------------------------------------------------------------
%  LOAD ROBOT PHYSICAL PARAMETERS
%  -------------------------------------------------------------------------
density;   % loads: g, alumd, ferd, pexid, plasd, Mch, kv, kp, Qi

%% -------------------------------------------------------------------------
%  LOAD SIMSCAPE MULTIBODY IMPORT DATA
%  -------------------------------------------------------------------------
DELTA_ROBOT_X_DataFile;   % populates smiData structure

%% -------------------------------------------------------------------------
%  GENERATE REFERENCE TRAJECTORY
%  -------------------------------------------------------------------------
t = 0:DT:T_END;
N = numel(t);

% Preallocate
xd_v  = zeros(1,N);  yd_v  = zeros(1,N);  zd_v  = zeros(1,N);
q1_v  = zeros(1,N);  q2_v  = zeros(1,N);  q3_v  = zeros(1,N);
dq1_v = zeros(1,N);  dq2_v = zeros(1,N);  dq3_v = zeros(1,N);
ddq1_v= zeros(1,N);  ddq2_v= zeros(1,N);  ddq3_v= zeros(1,N);

ttt = deg2rad(160);
Ti  = [ttt; ttt; ttt];

fprintf('Generating %s trajectory ...\n', upper(TRAJECTORY));
for i = 1:N
    switch lower(TRAJECTORY)
        case 'circle'
            xd = RADIUS * cos(OMEGA * t(i));
            yd = RADIUS * sin(OMEGA * t(i));
            zd = Z_OFFSET + Z_LIFT * t(i);
        case 'square'
            % Four-segment square (0.15 m half-side)
            seg = mod(floor(t(i)/2), 4);
            tau = mod(t(i), 2);
            switch seg
                case 0; xd = 0.075*tau - 0.15; yd =  0.075*tau;
                case 1; xd = 0.075*tau;         yd = -0.075*tau + 0.15;
                case 2; xd = 0.075*tau + 0.15;  yd = -0.075*tau;
                case 3; xd =-0.075*tau;          yd =  0.075*tau - 0.15;
            end
            zd = Z_OFFSET;
        otherwise
            error('Unknown TRAJECTORY type: %s', TRAJECTORY);
    end

    Tf = IGM(xd, yd, zd);
    [Tf(1), dTf(1), ddTf(1)] = quintic(Ti(1), Tf(1), 0,0,0,0, 0, DT*i, t(i));
    [Tf(2), dTf(2), ddTf(2)] = quintic(Ti(2), Tf(2), 0,0,0,0, 0, DT*i, t(i));
    [Tf(3), dTf(3), ddTf(3)] = quintic(Ti(3), Tf(3), 0,0,0,0, 0, DT*i, t(i));
    Ti = Tf;

    X = FGM(Tf(1), Tf(2), Tf(3));
    xd_v(i)=X(1); yd_v(i)=X(2); zd_v(i)=X(3);
    q1_v(i)=Tf(1);  q2_v(i)=Tf(2);  q3_v(i)=Tf(3);
    dq1_v(i)=dTf(1); dq2_v(i)=dTf(2); dq3_v(i)=dTf(3);
    ddq1_v(i)=ddTf(1); ddq2_v(i)=ddTf(2); ddq3_v(i)=ddTf(3);
end

% Package as Simulink timeseries
xdd  = timeseries(xd_v,  t);   ydd  = timeseries(yd_v,  t);   zdd  = timeseries(zd_v,  t);
q1d  = timeseries(q1_v,  t);   q2d  = timeseries(q2_v,  t);   q3d  = timeseries(q3_v,  t);
dq1d = timeseries(dq1_v, t);   dq2d = timeseries(dq2_v, t);   dq3d = timeseries(dq3_v, t);
ddq1d= timeseries(ddq1_v,t);   ddq2d= timeseries(ddq2_v,t);   ddq3d= timeseries(ddq3_v,t);

fprintf('Trajectory generation complete.\n');

%% -------------------------------------------------------------------------
%  SELECT AND RUN MODEL
%  -------------------------------------------------------------------------
model_map = struct( ...
    'backstepping', 'backstepping25072026', ...
    'pdsmc',        'DELTA_ROBOT_X_PDSMC25072026', ...
    'pdsmc_basic',  'PDplusSMC', ...
    'smc_linear',   'SlidingModeControllBasedOnlLinereasition' ...
);

if ~isfield(model_map, MODEL_CHOICE)
    error('Unknown MODEL_CHOICE: "%s". Choose from: backstepping, pdsmc, pdsmc_basic, smc_linear.', MODEL_CHOICE);
end

mdl_name = model_map.(MODEL_CHOICE);
mdl_path = fullfile(pwd, 'models', mdl_name);

fprintf('Opening model: %s\n', mdl_name);
open_system(mdl_path);

fprintf('Running simulation ...\n');
simOut = sim(mdl_name, 'StopTime', num2str(T_END));
fprintf('Simulation complete.\n');

%% -------------------------------------------------------------------------
%  BASIC RESULTS PLOT
%  -------------------------------------------------------------------------
figure('Name', sprintf('End-Effector Trajectory — %s', MODEL_CHOICE), 'NumberTitle', 'off');
plot3(xd_v, yd_v, zd_v, 'b--', 'LineWidth', 1.5); hold on;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title(sprintf('Delta Robot End-Effector Trajectory\nController: %s | Path: %s', ...
    upper(strrep(MODEL_CHOICE,'_',' ')), upper(TRAJECTORY)));
grid on; axis equal;
legend('Reference trajectory', 'Location', 'best');
view(3);
