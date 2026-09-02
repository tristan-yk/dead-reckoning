function [x, P, mem] = filter_init(x, P, mem, dt, sens_in)

    accel = sens_in.accel;
    gyro = sens_in.gyro;
    mag = sens_in.mag;
    baro = sens_in.baro;

    accel_filt = mem.sens_filt.accel;
    gyro_filt = mem.sens_filt.gyro;
    mag_filt = mem.sens_filt.mag;
    baro_filt = mem.sens_filt.baro;

    alpha_fast = 0.001;
    alpha_slow = 0.005;
    
    mem.sens_filt.accel = init_lowpass(accel, accel_filt, alpha_fast);
    mem.sens_filt.gyro = init_lowpass(gyro, gyro_filt, alpha_fast);
    mem.sens_filt.mag = init_lowpass(mag, mag_filt, alpha_slow);
    mem.sens_filt.baro = init_lowpass(mag, mag_filt, alpha_slow);

    x = zeros(10);
    x(1:4) = init_orientation(mem.sens_filt.accel, mem.sens_filt.mag);
    x(5:7) = mem.sens_filt.gyro;

    P = blkdiag( 1e-4*eye(4), ...      % attitude, ~1-4 deg
              1e-6*eye(3), ...      % gyro bias, ~1e-3 rad/s
              1e-2,        ...      % a_z, ~0.1 m/s^2
              1e-4,        ...      % v_z, ~0.01 m/s (static)
              1e-2         );       % h, ~0.1 m

end

