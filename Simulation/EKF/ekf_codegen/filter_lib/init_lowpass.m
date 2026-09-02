function [filt] = init_lowpass(sensor, filt, alpha)
    if sensor.status == true
        filt = (1 - alpha) * filt + alpha * sensor.meas;
    end
end