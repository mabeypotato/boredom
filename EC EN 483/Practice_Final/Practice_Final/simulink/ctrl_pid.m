function tau=ctrl_pid(in,P)
    theta_c = in(1);
    theta   = in(2);
    t       = in(3);
    
    tau = 
end

function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end