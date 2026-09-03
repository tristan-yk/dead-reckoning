function [x, P, mem] = filter_init(x, P, mem, dt, sens_in)

    persistent params
    if isempty(params)
        params = load("filter_params.mat");
    end

    accel = sens_in.accel;
    gyro = sens_in.gyro;
    mag = sens_in.mag;
    baro = sens_in.baro;

    accel_filt = mem.sens_filt.accel;
    gyro_filt = mem.sens_filt.gyro;
    mag_filt = mem.sens_filt.mag;
    baro_filt = mem.sens_filt.baro;

    alpha_fast = params.a_fast;
    alpha_slow = params.a_slow;
    
    mem.sens_filt.accel = init_lowpass(accel, accel_filt, alpha_fast);
    mem.sens_filt.gyro = init_lowpass(gyro, gyro_filt, alpha_fast);
    mem.sens_filt.mag = init_lowpass(mag, mag_filt, alpha_slow);
    mem.sens_filt.baro = init_lowpass(baro, baro_filt, alpha_slow);
    
    x = zeros(10);
    x(1:4) = init_orientation(mem.sens_filt.accel, mem.sens_filt.mag);
    x(5:7) = mem.sens_filt.gyro;

    P = params.P0;

end

