

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
% Sized from logged handling. Peak non-gravitational acceleration measured on
% the board is 5.3 m/s², reached in roughly a quarter of a second, so a random
% walk needs sa^2 * 0.25 >= 5.3^2, giving sa of about 10. An independent check
% agrees: even with sa held at 1, a_z was observed moving 2.35 m/s² across a
% single 40 ms sample, which alone implies sa near 12.
%
% The earlier values of 1 and 4 were both sized from the filter's own a_z
% estimate, which was itself being limited by sa - so they under-measured the
% real acceleration and confirmed their own assumption.

% innov
R_accel = 0.05 * eye(3);

% Fraction of gravity by which the accelerometer may deviate before it stops
% being treated as an attitude reference. Measured handling stays inside 10%
% about 94% of the time, while the tail runs to 54%, so this keeps nearly every
% usable sample and rejects the ones dominated by real acceleration.
accel_gate = 0.1;
R_mag = (3 * pi / 180)^2; % 3 degrees RMS

% Magnetic declination, radians, magnetic north measured east of true north.
% IGRF-14 at the plant's coordinates - 43.4643 N, 80.5204 W, 2025 - gives
% -9.4394 deg, i.e. 9.44 deg West. Without this the magnetometer innovation
% drives the field onto nav x, which silently makes the navigation frame
% magnetic rather than true north.
%
% This is location specific. Recompute with igrfmagm(0, lat, lon, year, 14) if
% the device is used somewhere else; the same value serves the simulation and
% the hardware only because both sit at these coordinates.
mag_declination = deg2rad(-9.4394);
R_baro = 4;    % 2 Pa RMS


save(fullfile(fileparts(mfilename('fullpath')), 'filter_params.mat'));