function q = init_orientation(d, mag)
    % NED

    d = - d / norm(d); 
    n = mag - dot(d, mag) * d;
    n = n / norm(n);
    e = cross(d, n);
    R_bn = [n e d];
    q = dcm2q(R_bn');

end