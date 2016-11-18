function [ avg_dist ] = avg_distance_bw_points( points )
    avg_sum = 0;
    for i = 1:size(points,1)
        x0 = repmat(points(i,:), size(points,1), 1);
        indiv_sum = sum(pdist2(x0, points, 'euclidean'));
        avg_sum = avg_sum + indiv_sum(1) / size(points,1);
    end
    
    avg_dist = avg_sum / size(points,1);
end

