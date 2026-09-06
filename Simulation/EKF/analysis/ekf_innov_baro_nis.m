function [x, P, y, S] = ekf_innov_baro_nis(x, P, baro, p0, R_override)
% Exact copy of ekf_innov_baro, with the innovation and its covariance also
% returned so the replay can compute NIS. The update itself is unchanged, so
% the replayed trajectory matches the flight filter by construction.

    persistent params
    if isempty(params)
        params = coder.load("filter_params.mat");
    end

    R = params.R_baro;
    if nargin > 4 && ~isempty(R_override), R = R_override; end

    L = 0.0065;
    T0 = 288.15;
    k = 5.2559;
    h = x(10);
    u = 1 - L * h / T0;
    z = p0 * u^k;
    H = zeros(1, 10, 'like', x);
    H(10) = -p0 * k * L / T0 * u^(k-1);

    y = baro - z;
    S = H * P * H' + R;
    K = P * H' / S;
    x = x + K * y;
    P = (eye(10) - K * H) * P * (eye(10) - K * H)' + K * R * K';

    [x, P] = ekf_norm(x, P);

end
