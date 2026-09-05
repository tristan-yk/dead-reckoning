
% Exercises filter_entry so MATLAB Coder can derive the entry-point input
% types from it. Every input is built as single precision on purpose: the
% generated code inherits its types from this script, and the Cortex-M4F on the
% target has a single-precision FPU, so a double-typed filter runs entirely in
% software floating point.
%
% The EKF sources themselves are type-neutral - they preallocate with
% zeros(..., 'like', x) - so they follow whatever this script passes in. Running
% the filter from Simulink still uses double, because filter_caller.m passes
% double state.

dt   = single(0.01);                        % 100 Hz base tick
g    = single(9.81);
mref = single([0.1864; 0; 0.4855]);         % Gauss, ~52 uT at 69 deg inclination
p0   = single(101325);

x = single([1; 0; 0; 0;  0; 0; 0;  0; 0; 0]);   % [q(4) b(3) a_z v_z h]
P = zeros(10, 10, 'single');

mem.sens_filt.accel = single([0; 0; -g]);
mem.sens_filt.gyro  = zeros(3, 1, 'single');
mem.sens_filt.mag   = mref;
mem.sens_filt.baro  = p0;

sens_in.accel.meas = single([0; 0; -g]);    sens_in.accel.status = true;
sens_in.gyro.meas  = zeros(3, 1, 'single'); sens_in.gyro.status  = true;
sens_in.mag.meas   = mref;                  sens_in.mag.status   = true;
sens_in.baro.meas  = p0;                    sens_in.baro.status  = true;

% 10 s idle (is_init = false -> filter_init), then 20 s of filtering.
for k = 0:2999
    t = k * dt;
    sens_in.accel.meas   = single([0; 0; -g] + 0.0082 * randn(3,1));
    sens_in.accel.status = true;                 % 100 Hz
    sens_in.gyro.meas    = single(0.0026 * randn(3,1));
    sens_in.gyro.status  = true;                 % 100 Hz
    sens_in.mag.meas     = single(mref + 0.006 * randn(3,1));
    sens_in.mag.status   = mod(k, 5) == 0;       %  20 Hz
    sens_in.baro.meas    = single(p0 + 2.0 * randn);
    sens_in.baro.status  = mod(k, 4) == 0;       %  25 Hz

    [x, P, mem] = filter_entry(x, P, mem, dt, sens_in, t >= 10);
end

fprintf('class   = %s\n',       class(x));
fprintf('q       = %s\n',       mat2str(x(1:4).', 5));
fprintf('bias    = %s rad/s\n', mat2str(x(5:7).', 3));
fprintf('a_z %+.4f  v_z %+.4f  h %+.4f\n', x(8), x(9), x(10));
fprintf('norm(P) = %.5f\n', norm(P));
