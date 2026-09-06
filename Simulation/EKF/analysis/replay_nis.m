function replay_nis(csv, R_baro_override, R_mag_override)
% Replay a hardware log through the flight filter and score its innovations.
%
% NIS = y' * inv(S) * y for each measurement update. For a consistent filter it
% is chi-squared with m degrees of freedom, so a scalar update should average 1.
% Above that the filter is overconfident, below it is conservative.
%
% Only the magnetometer and barometer are scored. The accelerometer's
% measurement model assumes the specific force is gravity, which is knowingly
% false whenever the board is moving - that is what the editing gate exists for
% - so its NIS would report how much the board was moved rather than anything
% about the filter.

    here = fileparts(mfilename('fullpath'));
    root = fullfile(here, '..', 'ekf_codegen');
    addpath(root, fullfile(root,'filter_lib'), fullfile(root,'filter_loop'), ...
            fullfile(root,'filter_init'), here);

    if nargin < 2, R_baro_override = []; end
    if nargin < 3, R_mag_override = []; end
    T = readmatrix(csv);
    ms=T(:,1); state=T(:,2); flags=T(:,3);
    acc=T(:,15:17); gyr=T(:,18:20); mg=T(:,21:23); bar=T(:,24);
    q_log=T(:,4:7); h_log=T(:,13);

    R_bs = [0 -1 0; -1 0 0; 0 0 -1];

    x = zeros(10,1,'single'); x(1)=1; P = zeros(10,10,'single');
    mem.sens_filt.accel=zeros(3,1,'single'); mem.sens_filt.gyro=zeros(3,1,'single');
    mem.sens_filt.mag=zeros(3,1,'single');   mem.sens_filt.baro=single(0);
    seeded=false(1,4); running=false; t_old=0;

    nis_mag=[]; nis_baro=[]; q_rep=nan(size(q_log)); h_rep=nan(size(h_log));

    for i = 1:numel(ms)
        s.accel.meas = single(R_bs*acc(i,:)');  s.accel.status = bitand(flags(i),1)>0;
        s.gyro.meas  = single(R_bs*gyr(i,:)');  s.gyro.status  = bitand(flags(i),2)>0;
        s.mag.meas   = single(R_bs*mg(i,:)');   s.mag.status   = bitand(flags(i),4)>0;
        s.baro.meas  = single(bar(i));          s.baro.status  = bitand(flags(i),8)>0;
        t = ms(i)/1000;

        if ~running
            if s.accel.status && ~seeded(1), mem.sens_filt.accel=s.accel.meas; seeded(1)=true; end
            if s.gyro.status  && ~seeded(2), mem.sens_filt.gyro =s.gyro.meas;  seeded(2)=true; end
            if s.mag.status   && ~seeded(3), mem.sens_filt.mag  =s.mag.meas;   seeded(3)=true; end
            if s.baro.status  && ~seeded(4), mem.sens_filt.baro =s.baro.meas;  seeded(4)=true; end
            if ~all(seeded), continue; end
            running=true; t_old=t;
        end

        dt = single(t - t_old);
        if s.gyro.status, t_old = t; end

        if state(i)==1                      % CALIBRATING -> filter_init
            [x,P,mem] = filter_init(x,P,mem,dt,s);
        elseif state(i)==2                  % RUNNING -> instrumented filter_loop
            if s.gyro.status,  [x,P] = ekf_dynamics(dt,x,P,s.gyro.meas); end
            if s.accel.status
                [x,P] = ekf_innov_accel(x,P,s.accel.meas,norm(mem.sens_filt.accel));
            end
            if s.mag.status
                [x,P,y,S] = ekf_innov_mag_nis(x,P,s.mag.meas,R_mag_override);
                nis_mag(end+1) = double(y)^2/double(S);              %#ok<AGROW>
            end
            if s.baro.status
                [x,P,y,S] = ekf_innov_baro_nis(x,P,s.baro.meas,mem.sens_filt.baro,R_baro_override);
                nis_baro(end+1) = double(y)^2/double(S);             %#ok<AGROW>
            end
            q_rep(i,:) = double(x(1:4))'; h_rep(i) = double(x(10));
        end
    end

    report('magnetometer', nis_mag);
    report('barometer   ', nis_baro);

    k = ~isnan(h_rep);
    dq = 2*acosd(min(1,abs(sum(q_rep(k,:).*q_log(k,:),2))));
    fprintf('\nreplay vs logged flight state (fidelity check):\n');
    fprintf('  attitude difference: mean %.3f deg, max %.3f deg\n', mean(dq), max(dq));
    fprintf('  altitude difference: mean %.4f m,  max %.4f m\n', ...
            mean(abs(h_rep(k)-h_log(k))), max(abs(h_rep(k)-h_log(k))));
end

function report(name, v)
    n = numel(v);
    if n==0, fprintf('%s: no updates\n', name); return; end
    lo = chi2inv(0.025, n)/n; hi = chi2inv(0.975, n)/n;
    verdict = 'CONSISTENT';
    if mean(v) > hi, verdict = 'OVERCONFIDENT (P too small)'; end
    if mean(v) < lo, verdict = 'CONSERVATIVE (P too large)'; end
    fprintf('%s  N=%5d  mean NIS %6.3f   95%% band [%.3f, %.3f]   %s\n', ...
            name, n, mean(v), lo, hi, verdict);
end
