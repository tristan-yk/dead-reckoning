function R_nb = q2dcm(q)
% Quaternion (Hamilton, scalar-first, body->nav) to rotation matrix.
% R_nb maps a body-frame vector into the nav frame: v_nav = R_nb * v_body
    q  = q(:) / norm(q);
    qw = q(1);
    qv = q(2:4);
    sk = [     0 -qv(3)  qv(2);
           qv(3)      0 -qv(1);
          -qv(2)  qv(1)      0];
    R_nb = (qw^2 - qv'*qv)*eye(3) + 2*(qv*qv') + 2*qw*sk;
end