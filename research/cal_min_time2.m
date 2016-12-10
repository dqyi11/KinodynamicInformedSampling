% returns the minimum time between 2 points ([x1,y1] and [x2,y2]) in the state space
% returns -1 if the solution does not exist
function T = cal_min_time2(x1, v1, x2, v2)
    global a_max
    dp_acc = 0.5*(v1+v2)*abs(v2-v1)/a_max;
    sigma = sign1(x2 - x1 - dp_acc);
    a2 = -sigma*a_max; 
    a1 = -a2;
    a = a1; b = 2*v1; c = (v2^2-v1^2)/(2*a2) - (x2-x1);
    q = -0.5*(b + sign1(b)*(b^2-4*a*c)^0.5);
    ta1_a = q/a; ta1_b = c/q;
    if (ta1_a > 0) ta1 = ta1_a;
    elseif (ta1_b > 0) ta1 = ta1_b;
    elseif (ta1_a == 0 || ta1_b == 0) T=0; return;
    else T=-1; return;
    end;
    T = (v2-v1)/a2 + 2*ta1;
end