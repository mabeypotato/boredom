function out=ctrl_obsv(in,P)
    theta_c  = in(1);
    theta    = in(2);
    t        = in(3);
    x        = in(4:5);
    
    
    out = [tau;xhat];
end
    
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end
    
