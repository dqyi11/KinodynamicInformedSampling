function plot_nd_kino_surf(results, no_dim)
    color = ['r','g','b','c','y','m','w','k'];
    for i=1:no_dim/2
        figure;
        for j=1:size(results,1)
            if size(results{j},1) == 0
                continue;
            end;
            plot(results{j}(:,2*i-1),results{j}(:,2*i),strcat(color(j),'*'));
            hold on;
        end
    end
end