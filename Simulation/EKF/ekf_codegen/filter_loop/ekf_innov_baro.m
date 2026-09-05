function [x, P] = ekf_innov_baro(x, P, baro, p0)

    persistent params
    if isempty(params)
        params = coder.load("filter_params.mat");
    end

    R = params.R_baro;
    
    % measurement prediction
    L = 0.0065;
    T0 = 288.15;
    k = 5.2559;
    h = x(10);
    u = 1 - L * h / T0;
    z = p0 * u^k;
    H = zeros(1, 10);
    H(10) = -p0 * k * L / T0 * u^(k-1);

    y = baro - z;
    S = H * P * H' + R;
    K = P * H' / S;
    x = x + K * y;
    P = (eye(10) - K * H) * P * (eye(10) - K * H)' + K * R * K';
    
    [x, P] = ekf_norm(x, P);

end


