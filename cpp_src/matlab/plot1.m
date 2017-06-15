data = sortrows(data05, 5);

data(:,1:5) = log(data(:,1:5))


figure;
hold on;
scatter(data(:,5),data(:,1)./data(:,6),'MarkerFaceColor','r', 'Marker', 'h');
scatter(data(:,5),data(:,2)./data(:,6),'MarkerFaceColor','g', 'Marker', 'd');
scatter(data(:,5),data(:,3)./data(:,6),'MarkerFaceColor','b', 'Marker', 'o');
scatter(data(:,5),data(:,4)./data(:,6),'MarkerFaceColor','c', 'Marker', 's');
xlabel('informed set volume ratio');
ylabel('time per sample (ms)');
legend('HMC', 'HRS', 'Hit&Run', 'Rejection');
hold off;

