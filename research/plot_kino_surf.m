function plot_kino_surf(x_range, y_range, cstfxn, T_best)
    global xx1 vv1 xx2 vv2;

    [X, Y] = meshgrid(x_range, y_range);
    Z = arrayfun(cstfxn, repmat(xx1,size(X)), repmat(vv1,size(Y)), repmat(xx2,size(X)), repmat(vv2,size(Y)), X, Y);

%     Z = arrayfun(cstfxn, [X(:), Y(:)], 2);
%     Z = rowfun(cstfxn, [X(:), Y(:)]);
%     if Ball == 1
%         Z = arrayfun(@cal_min_time2, repmat(x1,size(X)), repmat(v1,size(Y)), X, Y);
%     elseif Ball == 0
%         Z = arrayfun(@cal_min_time3, repmat(x1,size(X)), repmat(v1,size(Y)), repmat(x2,size(X)), repmat(v2,size(Y)), X, Y);
%     end
    contour3(X,Y,Z,'LevelList', [T_best], 'LineWidth', 2, 'Color', [0,0,0]); 
    hold on;
    surf(X,Y,Z, 'Edgecolor', 'none'); colorbar;
    xlabel('X'); ylabel('X_d_o_t'); zlabel('Time');
end