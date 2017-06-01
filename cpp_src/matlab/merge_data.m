

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