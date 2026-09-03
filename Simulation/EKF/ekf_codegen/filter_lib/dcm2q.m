function q = dcm2q(R_nb)
% Rotation matrix (body->nav) to quaternion (Hamilton, scalar-first).
% Shepperd's method: branch on the largest denominator for numerical stability.
    tr = trace(R_nb);
    if tr > 0
        s = sqrt(1 + tr) * 2;
        q = [0.25*s;
             (R_nb(3,2)-R_nb(2,3))/s;
             (R_nb(1,3)-R_nb(3,1))/s;
             (R_nb(2,1)-R_nb(1,2))/s];
    elseif R_nb(1,1) > R_nb(2,2) && R_nb(1,1) > R_nb(3,3)
        s = sqrt(1 + R_nb(1,1) - R_nb(2,2) - R_nb(3,3)) * 2;
        q = [(R_nb(3,2)-R_nb(2,3))/s;
             0.25*s;
             (R_nb(1,2)+R_nb(2,1))/s;
             (R_nb(1,3)+R_nb(3,1))/s];
    elseif R_nb(2,2) > R_nb(3,3)
        s = sqrt(1 + R_nb(2,2) - R_nb(1,1) - R_nb(3,3)) * 2;
        q = [(R_nb(1,3)-R_nb(3,1))/s;
             (R_nb(1,2)+R_nb(2,1))/s;
             0.25*s;
             (R_nb(2,3)+R_nb(3,2))/s];
    else
        s = sqrt(1 + R_nb(3,3) - R_nb(1,1) - R_nb(2,2)) * 2;
        q = [(R_nb(2,1)-R_nb(1,2))/s;
             (R_nb(1,3)+R_nb(3,1))/s;
             (R_nb(2,3)+R_nb(3,2))/s;
             0.25*s];
    end
    q = q / norm(q);
    if q(1) < 0, q = -q; end
end