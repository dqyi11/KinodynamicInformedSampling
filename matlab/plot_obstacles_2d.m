
obstacles_data = loadjson(['.' filesep 'obstacles.json']);

start = [-4, -4];
goal = [4, 4];

figure(11);
hold on;
rectangle('Position', [-5, -5. 10, 10], 'FaceColor', [1., 1., 1.]);
for i=1:1:size(obstacles_data.obstacles,2)
    obs = obstacles_data.obstacles{1,i};
    if obs.type == 'hypercube'
        rect = [obs.center(1,1)-obs.radius(1,1), obs.center(1,2)-obs.radius(1,2), 2*obs.radius(1,1),2*obs.radius(1,2)];
        rectangle('Position', rect,'FaceColor',[.0 .0 .0]);
    end
end
scatter(start(1,1), start(1,2), 'rs');
scatter(goal(1,1), goal(1,2), 'bs');
hold off;
%xticks(-5:1:5);
%yticks(-5:1:5);

