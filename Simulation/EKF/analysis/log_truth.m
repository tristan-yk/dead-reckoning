function L = log_truth(t, x, P, q_true, vert_true, bias_true)

    persistent T X PP Q V B n

    if nargin == 0
        if isempty(n) || n == 0
            error('log_truth:empty', 'nothing logged - run the model first');
        end
        L = struct('t', T(1:n), 'x', X(1:n,:), 'P', PP(1:n,:), ...
                   'q_true', Q(1:n,:), 'vert_true', V(1:n,:), ...
                   'bias_true', B(1:n,:));
        return
    end

    % Time going backwards means a new run and the buffer should restart. Time
    % repeating means the solver has called us twice for the same major step,
    % which is normal, and the sample is already stored - appending would
    % duplicate it and restarting would throw the whole run away.
    if ~isempty(n) && n > 0
        if t < T(n)
            n = 0;
        elseif t == T(n)
            return;
        end
    end

    if isempty(n) || n == 0
        cap = 4096;
        T  = zeros(cap,1);              X = zeros(cap, numel(x));
        PP = zeros(cap, numel(P));      Q = zeros(cap, numel(q_true));
        V  = zeros(cap, numel(vert_true));
        B  = zeros(cap, numel(bias_true));
        n  = 0;
    end

    if n == size(T,1)                    % grow by doubling
        T = [T; zeros(n,1)];             X  = [X;  zeros(n, size(X,2))];
        PP = [PP; zeros(n, size(PP,2))]; Q  = [Q;  zeros(n, size(Q,2))];
        V  = [V;  zeros(n, size(V,2))];  B  = [B;  zeros(n, size(B,2))];
    end

    n = n + 1;
    T(n)    = t;
    X(n,:)  = x(:)';
    PP(n,:) = P(:)';
    Q(n,:)  = q_true(:)';
    V(n,:)  = vert_true(:)';
    B(n,:)  = bias_true(:)';
end
