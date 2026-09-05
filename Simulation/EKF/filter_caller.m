function [attitude, altitude, P_norm] = filter_caller(time, accel, gyro, mag, baro)
    
    idle_time = 10;

    R_bs = [0 -1 0; -1 0 0; 0 0 -1];
    accel.meas = R_bs * accel.meas;
    gyro.meas  = R_bs * gyro.meas;
    mag.meas   = R_bs * mag.meas;

    persistent state P t_old mem

    if isempty(state)
        %% [q(4), b(3), a_z(1), v_z(1), h(1)]
        state = [[1; 0; 0; 0]; [0; 0; 0]; 0; 0; 0];
    end
    if isempty(P)
        P = zeros(10);
    end
    if isempty(t_old)
        t_old = time;
    end
    if isempty(mem)
        % take last known measurement anyway regardless of status
        mem.sens_filt.accel = accel.meas;
        mem.sens_filt.gyro = gyro.meas;
        mem.sens_filt.mag = mag.meas;
        mem.sens_filt.baro = baro.meas;
    end


    sens_in.accel = accel;
    sens_in.gyro = gyro;
    sens_in.mag = mag;
    sens_in.baro = baro;

    dt = time - t_old;
    if gyro.status == true
        t_old = time;
    end

    %% calibrate
    if time < idle_time
        ekf_active = false;
    else
        ekf_active = true;
    end

    [state, P, mem] = filter_entry(state, P, mem, dt, sens_in, ekf_active);

    attitude = state(1:4);
    altitude = state(10);
    P_norm = norm(P);

end
