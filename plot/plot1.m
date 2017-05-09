data = sortrows(test1, 6);

figure;
hold on;
scatter(data(:,6),data(:,1)./data(:,7),'MarkerFaceColor',[0 .7 .7]);
scatter(data(:,6),data(:,2)./data(:,7),'MarkerFaceColor',[.7 .0 .7]);
scatter(data(:,6),data(:,3)./data(:,7),'MarkerFaceColor',[.7 .7 .0]);
scatter(data(:,6),data(:,4)./data(:,7),'MarkerFaceColor',[.7 .0 .7]);
scatter(data(:,6),data(:,5)./data(:,7),'MarkerFaceColor',[.7 .7 .7]);
xlabel('informed set volume ratio');
ylabel('time per sample (ms)');
legend('MCMC', 'Rej', 'HRS', 'Gibbs','Hit&Run');
hold off;

index = 2
figure;
hold on;
bar(1:1:5, data(index,1:5))
xtickelables({'MCMC', 'Rej', 'HRS', 'Gibbs','Hit&Run'})
hold off;
