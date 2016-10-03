function UnitSphere
    a_max=1;
    x1=0; v1=1; x2=1; v2=-1;
    xc = (x1+x2)/2;
    vc = (v1+v2)/2;
    x = xc-6.0:0.04:xc+6.0;
    y = vc-3:0.04:vc+3;
    plot_surf();
    
    % returns the minimum time between 2 points ([x1,y1] and [x2,y2]) in the state space
    % returns -1 if the solution does not exist
    function T = cal_min_time2(x1, v1, x2, v2)
        dp_acc = 0.5*(v1+v2)*abs(v2-v1)/a_max;
        sigma = sign(x2 - x1 - dp_acc);
        a2 = -sigma*a_max; 
        a1 = -a2;
        a = a1; b = 2*v1; c = (v2^2-v1^2)/(2*a2) - (x2-x1);
        q = -0.5*(b + sign(b)*(b^2-4*a*c)^0.5);
        ta1_a = q/a; ta1_b = c/q;
        if (ta1_a > 0) ta1 = ta1_a;
        elseif (ta1_b > 0) ta1 = ta1_b;
        elseif (ta1_a == 0 || ta1_b == 0) T=0; return;
        else T=-1; return;
        end;
        T = (v2-v1)/a2 + 2*ta1;
    end

    % returns the minimum time between 3 points ([x1,y1] -> [xi,yi] -> [x2,y2]) in the state space
    % returns -1 if the solution does not exist
    function T = cal_min_time3(x1, v1, x2, v2, xi, vi)
        T1 = cal_min_time2(x1, v1, xi, vi);
        T2 = cal_min_time2(xi, vi, x2, v2);
        if (T1 < 0 || T2 < 0)
            T = -1;
            return;
        end
        T = T1 + T2;
    end

    function plot_surf()
        [X, Y] = meshgrid(x, y);
        %Z = arrayfun(@cal_min_time2, repmat(x1,size(X)), repmat(v1,size(Y)), X, Y);
        Z = arrayfun(@cal_min_time3, repmat(x1,size(X)), repmat(v1,size(Y)), repmat(x2,size(X)), repmat(v2,size(Y)), X, Y);
%         contour3(X,Y,Z,10,'k'); 
%         hold on; surf(X,Y,Z, 'Edgecolor', 'none'); colorbar; hold off;
%         xlabel('X'); ylabel('X_d_o_t'); zlabel('Time');
        figure;
        plot(x1,v1,'r*','markers',12); hold on; plot(x2,v2,'K*','markers',12); hold off;
        hold on; pcolor(X,Y,Z); shading flat; colorbar; hold off;
        hold on; contour(X,Y,Z,2,'k'); hold off;
        xlabel('X'); ylabel('X_d_o_t'); zlabel('Time');
        hold on; plot(x1,v1,'r*','markers',12); plot(x2,v2,'K*','markers',12); hold off;
        legend('start','goal');
    end

    function s = sign(x)
        if (x>=0) s=1; else s=-1; end
    end
end