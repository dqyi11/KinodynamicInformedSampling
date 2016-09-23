function UnitSphere
x1=10;
v1=5;
a_max=1;
a1=a_max;
a2=-a_max;

% returns the minimum time between 2 points ([x1,y1] and [x2,y2]) in the state space
% returns -1 if the solution does not exist
function T = cal_min_time(x2, v2)
    dp_acc = 0.5*(v1+v2)*abs(v2-v1)/a_max;
    sigma = sign(x2 - x1 - dp_acc);
    a2 = -sigma*a_max;
    a1 = -a2;
    ta1_a = (-2*v1 + (v1^2 - 4*a1*((v2^2-v1^2)/(2*a2) - (x2-x1))))/(2*a1);
    ta1_b = (-2*v1 - (v1^2 - 4*a1*((v2^2-v1^2)/(2*a2) - (x2-x1))))/(2*a1);
    if ta1_a > 0
        ta1 = ta1_a;
    elseif ta1_a == 0 && ta1_b < 0
        ta1 = ta1_a;
    elseif ta1_b >= 0
        ta1 = ta1_b;
    else
        ta1=-100; %random large negative value
    end
    T = (v2-v1)/a2 + 2*ta1;
end

x = x1-15.0:0.03:x1+15.0;
y = v1-15.0:0.03:v1+4.0;
[X, Y] = meshgrid(x, y);
Z = arrayfun(@cal_min_time, X, Y);
h = surf(X,Y,Z);
set(h, 'edgecolor','none')
colorbar;
[Xc,Yc,Zc] = cylinder;
hold on;
surf(0.05*Xc+x1,0.05*Yc+v1,100*abs(Zc));
hold off;
xlabel('X');
ylabel('X_d_o_t');
zlabel('Time');
end