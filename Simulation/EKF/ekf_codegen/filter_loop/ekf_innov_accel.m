function [x, P] = ekf_innov_accel(x, P, accel, g_magnitude)
    
    persistent params
    if isempty(params)
        params = coder.load("filter_params.mat");
    end

    R = params.R_accel;

    q = x(1:4);
    qw = q(1);
    qv = q(2:4);

    a_nav = [0; 0; - x(8) - g_magnitude];
    R_bn = q2dcm(q)';

    z = R_bn * a_nav;
    
    a_nav_cross = [       0 -a_nav(3)  a_nav(2);
                   a_nav(3)        0  -a_nav(1);
                  -a_nav(2)  a_nav(1)        0];

    H = zeros(3, 10, 'like', x);
    H(:,1)   = 2 * qw * a_nav - 2 * cross(qv, a_nav);
    H(:,2:4) = -2 * a_nav * qv' + 2 * (qv' * a_nav) * eye(3) + 2 * qv *a_nav' + 2 * qw * a_nav_cross;
    H(:,8)   = - R_bn(:,3);

    dev   = abs(norm(accel) - g_magnitude);
    R = R * (1 + (dev/0.5)^2);

    y = accel - z;
    S = H * P * H' + R;
    K = P * H' / S;

    x = x + K * y;
    P = (eye(10) - K * H) * P * (eye(10) - K * H)' + K * R * K';

    [x, P] = ekf_norm(x, P);

end