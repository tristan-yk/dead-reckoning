

% filter init
P0 = blkdiag(1e-4*eye(4), ...      % attitude, ~1-4 deg
             1e-6*eye(3), ...      % gyro bias, ~1e-3 rad/s
             1e-2,        ...      % a_z, ~0.1 m/s^2
             1e-4,        ...      % v_z, ~0.01 m/s (static)
             1e-2         );       % h, ~0.1 m

a_fast = 0.01;
a_slow = 0.04;


% dynamics
sg  = 1e-3;    % gyro noise density,     rad/s/√Hz
sbg = 1e-4;    % gyro bias walk,         rad/s/√s
sa  = 10;      % vertical accel disturb,   m/s²/√s

% innov
R_accel = 0.05 * eye(3);

accel_gate = 0.1;
R_mag = (3 * pi / 180)^2; % 3 degrees RMS

mag_declination = deg2rad(-9.4394);
R_baro = 1.5;  % measured 0.94 Pa RMS, inflated for un-modelled drift


save(fullfile(fileparts(mfilename('fullpath')), 'filter_params.mat'));