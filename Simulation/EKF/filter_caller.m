function [attitude, altitude, P_norm] = ekf_caller(time, accel, gyro, mag, baro)
    idle_time = 10;

    persistent state P t_old g

    if isempty(state)
        %% [q(4), b(3), a_z(1), v_z(1), h(1)]
        state = [[1; 0; 0; 0]; [0; 0; 0]; 0; 0; 0];
    end
    if isempty(P)
        P = zeros(9);
    end
    if isempty(t_old)
        t_old = time;
    end
    if isempty(g)
        g = [0; 0; 0];
    end
    
    dt = time - t_old;
    t_old = time;

    %% calibrate
    if time < idle_time
        ekf_active = false;
    end

    %% filter
    if time >= idle_time
        ekf_active = true;
    end

    [state, P, g] = ekf(dt, ekf_active, state, P, g, accel, gyro, mag, baro);

    attitude = state(1:4);
    altitude = 0; %state(9);
    P_norm = 0; %norm(P);

end
