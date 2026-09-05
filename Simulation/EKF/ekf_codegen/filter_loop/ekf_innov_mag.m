function [x, P] = ekf_innov_mag(x, P, mag)

    persistent params
    if isempty(params)
        params = coder.load("filter_params.mat");
    end

    R = params.R_mag;
    
    q = x(1:4);
    qw = q(1);
    qv = q(2:4);

    R_nb = q2dcm(q);

    m_nav = R_nb * mag;
    d = m_nav(1)^2 + m_nav(2)^2;
    % d_psi/dm
    dpsi = [-m_nav(2)/d m_nav(1)/d 0];

    m_cross = [      0 -mag(3) mag(2);
                mag(3)      0 -mag(1);
               -mag(2)  mag(1)      0];
    
    dm = zeros(3, 4);
    dm(:,1)   = 2 * qw * mag + 2 * cross(qv, mag);
    dm(:,2:4) = -2 * mag * qv' + 2 * (qv' * mag) * eye(3) + 2 * qv * mag' - 2 * qw * m_cross;

    H = zeros(1, 10);
    H(1:4) = dpsi * dm;
    R = R * (mag' * mag) / d; 

    % nav x = magnetic north
    psi = atan2(m_nav(2), m_nav(1));
    
    y = atan2(sin(-psi), cos(-psi)); % wrap to (-pi, pi]
    S = H * P * H' + R;
    K = P * H' / S;

    x = x + K * y;
    P = (eye(10) - K * H) * P * (eye(10) - K * H)' + K * R * K';

    [x, P] = ekf_norm(x, P);

end