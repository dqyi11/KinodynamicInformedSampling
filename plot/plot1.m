data = sortrows(data05, 5);

data(:,1:5) = log(data(:,1:5))


figure;
hold on;
scatter(data(:,5),data(:,1)./data(:,6),'MarkerFaceColor',[0 .7 .7]);
scatter(data(:,5),data(:,2)./data(:,6),'MarkerFaceColor',[.7 .0 .7]);
scatter(data(:,5),data(:,3)./data(:,6),'MarkerFaceColor',[.7 .0 .0]);
scatter(data(:,5),data(:,4)./data(:,6),'MarkerFaceColor',[.7 .7 .0]);
xlabel('informed set volume ratio');
ylabel('time per sample (ms)');
legend('HMC', 'HRS', 'Hit&Run', 'Rejection');
hold off;

