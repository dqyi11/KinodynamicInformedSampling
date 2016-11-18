%% Function for plotting the eudlidean distance curve
function [Z] = plot_surf(x_range, y_range, cstfxn)
    [X,Y] = meshgrid(x_range,y_range);
    sizex = size(X);
    sizey = size(Y);
    size_mesh = size(X);
    Z = feval(cstfxn, [X(:),Y(:)]);
    X = reshape(X, size_mesh);
    Y = reshape(Y, size_mesh);
    Z = reshape(Z, size_mesh);

    surf(X,Y,Z, 'edgecolor', 'none');
end