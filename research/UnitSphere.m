function UnitSphere
x1=0;
v1=0;
a_max=1;

% returns the minimum time between 2 points ([x1,y1] and [x2,y2]) in the state space
% returns -1 if the solution does not exist
function T = cal_min_time(x2, v2)
    dp_acc = 0.5*(v1+v2)*abs(v2-v1)/a_max;
    sigma = sign(x2 - x1 - dp_acc);
    a2 = -sigma*a_max;
    a1 = -a2;
    a = a1;
    b = 2*v1;
    c = (v2^2-v1^2)/(2*a2) - (x2-x1);
    q = -0.5*(b + sign(b)*(b^2-4*a*c)^0.5);
    ta1_a = q/a; ta1_b = c/q;
    if(ta1_a >= 0);
        ta1 = ta1_a;
    elseif (ta1_b >= 0)
        ta1 = ta1_b;
    elseif (ta1_a==0 && ta1_b==0)
        T=0;
        return;
    else 
        T=-1;
        return;
    end
    T = (v2-v1)/a2 + 2*ta1;
    if T<0
        disp('stuff');
    end
end

x = x1-3.0:0.02:x1+3.0;
y = v1-3.0:0.02:v1+3.0;
[X, Y] = meshgrid(x, y);
Z = arrayfun(@cal_min_time, X, Y);
h = surf(X,Y,Z);
set(h, 'edgecolor','none')
colorbar;
[Xc,Yc,Zc] = cylinder;
hold on;
surf(0.05*Xc+x1,0.05*Yc+v1,1*abs(Zc));
hold off;
xlabel('X');
ylabel('X_d_o_t');
zlabel('Time');
figure;
contour(X,Y,Z);
end