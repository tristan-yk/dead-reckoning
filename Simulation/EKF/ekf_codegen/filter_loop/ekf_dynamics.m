function [x, P] = ekf_dynamics(dt, x, P, gyro_meas)
    %#codegen

    persistent params

    if isempty(params)
        params = coder.load("filter_params.mat");
    end

    sg = params.sg;
    sbg = params.sbg;
    sa = params.sa;
    
    q = x(1:4);
    b = x(5:7);
    a = x(8);
    v = x(9);
    h = x(10);

    w = gyro_meas - b;
    theta = w * dt;
    phi = norm(theta);

    if phi < 1e-8
        dq  = [1; theta/2];
        Psi = [-theta' / 4; 0.5 * eye(3)]; % numerical stability
    else
        n = theta / phi;  
        sp = sin(phi/2);  
        cp = cos(phi/2);
        dq  = [cp; n * sp];
        Psi = [                                      -0.5 * sp * n'; 
               (sp / phi) * (eye(3) - n * n') + 0.5 * cp * (n * n')];
    end

    % state update
    qn = [q(1)*dq(1) - q(2:4)'*dq(2:4);
          q(1)*dq(2:4) + dq(1)*q(2:4) + cross(q(2:4), dq(2:4))];
    x = [qn; b; a; v + a * dt; h + v * dt + 0.5 * a * dt^2];


    Omega_tilde = [   0 -w(1) -w(2) -w(3);
                   w(1)     0  w(3) -w(2);
                   w(2) -w(3)     0  w(1);
                   w(3)  w(2) -w(1)     0];

    Q_tilde = [q(1) -q(2) -q(3) -q(4);
               q(2)  q(1) -q(4)  q(3);
               q(3)  q(4)  q(1) -q(2);
               q(4) -q(3)  q(2)  q(1)];

   
    % dynamics jacobian
    F = zeros(10, 'like', x);
    F(1:4, 1:4) = expm(0.5 * dt * Omega_tilde); % q
    F(1:4, 5:7) = -dt * Q_tilde * Psi; %q bias update
    F(5:7, 5:7) = eye(3); % b
    F(8, 8) = 1; % a_z
    F(9, 8:9) = [dt 1]; % v_z
    F(10, 8:10) = [0.5*dt^2 dt 1]; % h

    % model noise jacobian
    G = zeros(10, 7, 'like', x);
    G(1:4,1:3) = F(1:4,5:7);
    G(5:7,4:6) = eye(3);
    G(8,7)     = 1;

    Qc = diag([sg^2*[1 1 1] sbg^2*[1 1 1] sa^2]);
    Q = G*Qc*G'*dt;
    Q(8:10,8:10) = sa^2 * [dt     dt^2/2 dt^3/6;
                           dt^2/2 dt^3/3 dt^4/8;
                           dt^3/6 dt^4/8 dt^5/20]; % a_z noise affects h, in finite dt

    Q(1:4,1:4) = Q(1:4,1:4) + 1e-12*eye(4); % increase Q to compensate no ES EKF

    P = F * P * F' + Q;
    
    [x, P] = ekf_norm(x, P);
end