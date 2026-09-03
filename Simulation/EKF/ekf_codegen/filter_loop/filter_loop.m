function [x, P, mem] = filter_loop(x, P, mem, dt, sens_in)
    
    accel = sens_in.accel;
    gyro = sens_in.gyro;
    mag = sens_in.mag;
    baro = sens_in.baro;

    if gyro.status == true
        [x, P] = ekf_dynamics(dt, x, P, gyro.meas);
    end

    if accel.status == true
        g_magnitude = norm(mem.sens_filt.accel);
        [x, P] = ekf_innov_accel(x, P, accel.meas, g_magnitude);
    end

    if mag.status == true
        [x, P] = ekf_innov_mag(x, P, mag.meas);
    end

    if baro.status == true
        p0 = mem.sens_filt.baro;
        [x, P] = ekf_innov_baro(x, P, baro.meas, p0);
    end

end