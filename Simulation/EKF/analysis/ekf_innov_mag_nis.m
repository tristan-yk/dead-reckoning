function [x, P, y, S] = ekf_innov_mag_nis(x, P, mag, R_override)
% Exact copy of ekf_innov_mag with the innovation and its covariance returned.
% Note R is scaled by (mag'*mag)/d: the heading measurement is an angle formed
% from the horizontal projection, so its effective noise depends on how much of
% the field is left after projecting out the vertical component.

    persistent params
    if isempty(params)
        params = coder.load("filter_params.mat");
    end

    R = params.R_mag;
    if nargin > 3 && ~isempty(R_override), R = R_override; end

    q = x(1:4);
    qw = q(1);
    qv = q(2:4);

    R_nb = q2dcm(q);

    m_nav = R_nb * mag;
    d = m_nav(1)^2 + m_nav(2)^2;
    dpsi = [-m_nav(2)/d m_nav(1)/d 0];

    m_cross = [      0 -mag(3) mag(2);
                mag(3)      0 -mag(1);
               -mag(2)  mag(1)      0];

    dm = zeros(3, 4, 'like', x);
    dm(:,1)   = 2 * qw * mag + 2 * cross(qv, mag);
    dm(:,2:4) = -2 * mag * qv' + 2 * (qv' * mag) * eye(3) + 2 * qv * mag' - 2 * qw * m_cross;

    H = zeros(1, 10, 'like', x);
    H(1:4) = dpsi * dm;
    R = R * (mag' * mag) / d;

    D = params.mag_declination;
    psi = atan2(m_nav(2), m_nav(1));

    e = D - psi;
    y = atan2(sin(e), cos(e));
    S = H * P * H' + R;
    K = P * H' / S;

    x = x + K * y;
    P = (eye(10) - K * H) * P * (eye(10) - K * H)' + K * R * K';

    [x, P] = ekf_norm(x, P);

end
