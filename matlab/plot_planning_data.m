
hmc_filename = 'simple_HMC_0.csv';
hnr_filename = 'simple_HNR_0.csv';
rs_filename = 'simple_RS_0.csv';
hrs_filename = 'simple_HRS_0.csv';
HMC = csvread(hmc_filename);
HNR = csvread(hnr_filename);
RS = csvread(rs_filename);
HRS = csvread(hrs_filename);

figure;
hold on;
plot(log(HMC(:,1)), log(HMC(:,2)), 'o-r');
plot(log(HNR(:,1)), log(HNR(:,2)), 'o-g');
plot(log(RS(:,1)), log(RS(:,2)), 'o-b');
plot(log(HRS(:,1)), log(HRS(:,2)), 'o-c');
legend('HMC', 'HNR', 'RS', 'HRS');
xlabel('Time - log(ms)');
ylabel('Cost - log');
hold off;

allTime = vertcat(HMC(:,1), HNR(:,1), RS(:,1), HRS(:,1));
maxT = max(allTime);

stepNum = 500;
stepSize = log(maxT) / stepNum;

T = [0:stepSize:log(maxT)];
T = exp(T);
HMC1 = resample_data(HMC(:,2), HMC(:,1), T');
HNR1 = resample_data(HNR(:,2), HNR(:,1), T');
RS1 = resample_data(RS(:,2), RS(:,1), T');
HRS1 = resample_data(HRS(:,2), HRS(:,1), T');

figure;
hold on;
plot(log(T), log(HMC1), '.-r');
plot(log(T), log(HNR1), '.-g');
plot(log(T), log(RS1), '.-b');
plot(log(T), log(HRS1), '.-c');
legend('HMC', 'HNR', 'RS', 'HRS');
xlabel('Time - log(ms)');
ylabel('Cost - log');
hold off;