data = sortrows(test1, 6);

figure;
hold on;
scatter(data(:,6),data(:,1)./data(:,7),'MarkerFaceColor',[0 .7 .7]);
scatter(data(:,6),data(:,2)./data(:,7),'MarkerFaceColor',[.7 .0 .7]);
scatter(data(:,6),data(:,3)./data(:,7),'MarkerFaceColor',[.7 .7 .0]);
scatter(data(:,6),data(:,4)./data(:,7),'MarkerFaceColor',[.7 .0 .7]);
scatter(data(:,6),data(:,5)./data(:,7),'MarkerFaceColor',[.7 .0 .7]);
legend('MCMC', 'Rej', 'HRS', 'Gibbs','Hit&Run');
hold off;
