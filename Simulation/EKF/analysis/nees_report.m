function nees_report(log, opts)
% Score filter consistency against plant truth using NEES.
%
% NEES = (x - xhat)' * inv(P) * (x - xhat), chi-squared with n degrees of
% freedom for a consistent filter, so it should average n. Above that the
% filter is overconfident, below it is conservative. Unlike NIS this needs
% truth, so it only works in simulation - but it sees states the measurements
% never touch directly, which NIS cannot.
%
% log must contain, one row per sample:
%   t          N x 1    time
%   x          N x 10   filter state, from filter_caller
%   P          N x 100  filter covariance, P(:)' each step
%   q_true     N x 4    plant attitude, Hamilton scalar-first body->nav
%   vert_true  N x 3    plant [a_z v_z h], or N x 2 for [v_z h]
%   bias_true  N x 3 or 1 x 3   plant gyro bias, in SENSOR axes
%
% opts (all optional):
%   t_start    ignore samples before this time, default 12 s, to skip
%              filter_init and the settling transient
%   flip_vert  negate vert_true, default true - the plant is positive down
%              and the filter's vertical channel is positive up
%   rotate_bias  apply R_bs to bias_true, default true - filter_caller rotates
%              the measurements, so a bias injected in sensor axes reaches the
%              filter already rotated

    if nargin < 2, opts = struct; end
    if ~isfield(opts,'t_start'),     opts.t_start = 12;      end
    if ~isfield(opts,'flip_vert'),   opts.flip_vert = true;  end
    if ~isfield(opts,'rotate_bias'), opts.rotate_bias = true; end

    R_bs = [0 -1 0; -1 0 0; 0 0 -1];

    need = {'t','x','P','q_true','vert_true','bias_true'};
    for f = need
        if ~isfield(log, f{1})
            error('nees_report:missing', 'log is missing field "%s"', f{1});
        end
    end

    % The logger runs at the solver step, which is far faster than the filter,
    % so the state is held constant across many rows. Those repeats carry no new
    % information, and counting them would shrink the chi-squared band by
    % sqrt(oversampling) and condemn a healthy filter. Keep only rows where the
    % estimate actually moved.
    if ~isfield(opts,'decimate') || opts.decimate
        fresh = [true; any(abs(diff(log.x,1,1)) > 0, 2)];
        if sum(fresh) < numel(fresh)
            fprintf('decimating %d rows to %d filter updates (%.0fx oversampled)\n', ...
                    numel(fresh), sum(fresh), numel(fresh)/sum(fresh));
            for f = {'t','x','P','q_true','vert_true','bias_true'}
                if size(log.(f{1}),1) == numel(fresh)
                    log.(f{1}) = log.(f{1})(fresh,:);
                end
            end
        end
    end

    k = log.t >= opts.t_start;
    t = log.t(k); X = log.x(k,:); PP = log.P(k,:);
    QT = log.q_true(k,:); VT = log.vert_true(k,:);
    BT = log.bias_true;
    if size(BT,1) == 1, BT = repmat(BT, sum(k), 1); else, BT = BT(k,:); end

    nv = size(VT,2);                 % 3 for [a_z v_z h], 2 for [v_z h]
    vidx = (11-nv):10;               % 8:10 or 9:10

    % Checked before the sign flip, or a truth port fed from the estimate would
    % look merely negated rather than identical.
    vert_is_estimate = isequal(VT, X(:,vidx));

    if opts.flip_vert,   VT = -VT;        end
    if opts.rotate_bias, BT = (R_bs*BT')'; end
    N = numel(t);
    e_att = zeros(N,1); e_bias = zeros(N,1); e_vert = zeros(N,1);
    ang = zeros(N,1);

    for i = 1:N
        P = reshape(PP(i,:), 10, 10);
        P = (P + P')/2;                                  % kill asymmetry drift

        qh = X(i,1:4)'; qt = QT(i,:)';
        if dot(qh,qt) < 0, qt = -qt; end                 % q and -q are one rotation
        dq = qmul(qconj(qh), qt);
        a  = 2*dq(2:4);                                  % small-angle error, rad
        ang(i) = norm(a);
        w = qh(1); v = qh(2:4);
        J = 2*[-v, (w*eye(3) - skew(v))];                % d(a)/d(q), 3 x 4
        Pa = J*P(1:4,1:4)*J';
        e_att(i) = a' * (Pa \ a);

        eb = X(i,5:7)' - BT(i,:)';
        e_bias(i) = eb' * (P(5:7,5:7) \ eb);

        ev = X(i,vidx)' - VT(i,:)';
        e_vert(i) = ev' * (P(vidx,vidx) \ ev);
    end

    fprintf('\nsanity checks (get these right before trusting the numbers)\n');
    fprintf('  attitude error at t=%.1f s: %.3f deg', t(1), rad2deg(ang(1)));
    if rad2deg(ang(1)) > 10
        fprintf('   <-- LARGE. quaternion convention or hemisphere is wrong\n');
    else
        fprintf('\n');
    end
    fprintf('  filter h vs truth h correlation: %+.3f', corr(X(:,10), VT(:,end)));
    if corr(X(:,10), VT(:,end)) < 0
        fprintf('   <-- NEGATIVE. flip_vert is set the wrong way\n');
    else
        fprintf('\n');
    end
    fprintf('  bias error, mean magnitude: %.5f rad/s\n', mean(vecnorm(X(:,5:7)-BT,2,2)));

    fprintf('\n%d samples from %.1f to %.1f s\n', N, t(1), t(end));
    report('attitude  ', e_att,  3);
    report('gyro bias ', e_bias, 3);
    if vert_is_estimate
        fprintf(['vertical    SKIPPED - vert_true is bit-identical to the ' ...
                 'estimate, so the\n            filter is being scored ' ...
                 'against itself. Wire the plant truth\n            into that ' ...
                 'port, not the filter output.\n']);
    else
        report('vertical  ', e_vert, nv);
    end
end

function report(name, v, dof)
    n = numel(v);
    lo = chi2inv(0.025, n*dof)/n; hi = chi2inv(0.975, n*dof)/n;
    m = mean(v);
    verdict = 'CONSISTENT';
    if m > hi, verdict = 'OVERCONFIDENT (P too small)'; end
    if m < lo, verdict = 'CONSERVATIVE (P too large)';  end
    fprintf('%s dof=%d  mean NEES %7.3f   expected %d   95%% band [%.3f, %.3f]   %s\n', ...
            name, dof, m, dof, lo, hi, verdict);
end

function r = qmul(p, q)
    r = [p(1)*q(1) - p(2:4)'*q(2:4);
         p(1)*q(2:4) + q(1)*p(2:4) + cross(p(2:4), q(2:4))];
end

function c = qconj(q), c = [q(1); -q(2:4)]; end

function S = skew(v)
    S = [    0 -v(3)  v(2);
          v(3)     0 -v(1);
         -v(2)  v(1)     0];
end
