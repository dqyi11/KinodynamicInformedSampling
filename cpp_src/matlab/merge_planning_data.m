datasetSize = 13;
maxtimes = zeros(datasetSize);
for i = 0:1:12
  hmc_filename = sprintf('data/simple_HMC_%d.csv', i);
  hnr_filename = sprintf('data/simple_HNR_%d.csv', i);
  rs_filename = sprintf('data/simple_RS_%d.csv', i);
  hrs_filename = sprintf('data/simple_HRS_%d.csv', i);
  
  HMC = csvread(hmc_filename);
  HNR = csvread(hnr_filename);
  RS = csvread(rs_filename);
  HRS = csvread(hrs_filename);
  
  allTime = vertcat(HMC(:,1), HNR(:,1), RS(:,1), HRS(:,1));
  
  maxtimes(i+1) = max(allTime);
end

maxT = max(maxtimes);

stepNum = 10;
HMC1 = zeros(stepNum+1, datasetSize);
HNR1 = zeros(stepNum+1, datasetSize);
RS1 = zeros(stepNum+1, datasetSize);
HRS1 = zeros(stepNum+1, datasetSize);

stepSize = log(maxT) / stepNum;
T = [0:stepSize:log(maxT)];
T = exp(T);

for i = 0:1:12
  hmc_filename = sprintf('data/simple_HMC_%d.csv', i);
  hnr_filename = sprintf('data/simple_HNR_%d.csv', i);
  rs_filename = sprintf('data/simple_RS_%d.csv', i);
  hrs_filename = sprintf('data/simple_HRS_%d.csv', i);
  
  HMC = csvread(hmc_filename);
  HNR = csvread(hnr_filename);
  RS = csvread(rs_filename);
  HRS = csvread(hrs_filename);

  
  HMC1(:,i+1) = resample_data(HMC(:,2), HMC(:,1), T');
  HNR1(:,i+1) = resample_data(HNR(:,2), HNR(:,1), T'); 
  RS1(:,i+1) = resample_data(RS(:,2), RS(:,1), T');
  HRS1(:,i+1) = resample_data(HRS(:,2), HRS(:,1), T');
end
  
figure;
hold on;
plot(log(T), mean(log(HMC1),2),'-.r');
plot(log(T), mean(log(HNR1),2), '.-g');
plot(log(T), mean(log(RS1),2), '.-b');
plot(log(T), mean(log(HRS1),2), '.-c');
legend('HMC', 'HNR', 'RS', 'HRS');
xlabel('Time - log(ms)');
ylabel('Cost - log');
hold off;