% Function for plotting the eudlidean distance curve
% To be used with my_dist3
function [Z] = plot_surf3(x1, x2, x_range, y_range, my_dist3)
    [X,Y] = meshgrid(x_range,y_range);
    size_mesh = size(X);
    Z = feval(my_dist3, x1, x2, [X(:),Y(:)]);
    X = reshape(X, size_mesh);
    Y = reshape(Y, size_mesh);
    Z = reshape(Z, size_mesh);
    surf(X,Y,Z, 'edgecolor', 'none');
end