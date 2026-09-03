function [x, P] = ekf_norm(x, P)
    
    x(1:4) = x(1:4) / norm(x(1:4));
    P = (P + P') / 2;

end
