function [x, P, mem] = filter_entry(x, P, mem, dt, sens_in, is_init)
    %#codegen
    
    if is_init == false
        [x, P, mem] = filter_init(x, P, mem, dt, sens_in);
    else
        [x, P, mem] = filter_loop(x, P, mem, dt, sens_in);
    end


end