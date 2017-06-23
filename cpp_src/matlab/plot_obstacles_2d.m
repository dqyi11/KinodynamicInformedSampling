


obs_cube_center = [0.0, 0.0; 2.0, 1.2; -2.0, 1.2; -2.0, -1.2; 2.0, -1.2];
obs_cube_center1 = [-2. 3.5; -2, -3.5; 2, 3.5; 2, -3.5];
obs_cube_center2 = [-4, 2.; -4, -2.; 4., 2.; 4., -2];
obs_cube_center3 = [0.0, -3.; 0.0, 3.];
obs_cube_radius = [1.0, 1.0; 0.5, 0.5; 0.5, 0.5; 0.5, 0.5; 0.5, 0.5];
obs_cube_radius1 = [0.5, 0.5; 0.5, 0.5; 0.5, 0.5; 0.5, 0.5];
obs_cube_radius2 = [0.5, 0.5; 0.5, 0.5; 0.5, 0.5; 0.5, 0.5];
obs_cube_radius3 = [0.5, 1.0; 0.5, 1.0];

obs_cube_center = [obs_cube_center; obs_cube_center1; obs_cube_center2; obs_cube_center3];
obs_cube_radius = [obs_cube_radius; obs_cube_radius1; obs_cube_radius2; obs_cube_radius3];

num = 5+4+4+2;

start = [-4, -4];
goal = [4, 4];

figure(11);
hold on;
rectangle('Position', [-5, -5. 10, 10], 'FaceColor', [1., 1., 1.]);
for i=1:num
    rect = [obs_cube_center(i,1)-obs_cube_radius(i,1), obs_cube_center(i,2)-obs_cube_radius(i,2), 2*obs_cube_radius(i,1),2*obs_cube_radius(i,2)];
    rectangle('Position', rect,'FaceColor',[.0 .0 .0]);
end
scatter(start(1,1), start(1,2), 'rs');
scatter(goal(1,1), goal(1,2), 'bs');
hold off;
%xticks(-5:1:5);
%yticks(-5:1:5);

