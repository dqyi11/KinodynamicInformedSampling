clear; clc; close all;
hmc = dlmread('test1_hmc.log');
rej = dlmread('test1_rej.log');

for i = 1:2:(size(hmc,2)-1)
    figure; hold on;
    plot(hmc(:,i),hmc(:,i+1),'b.');
    plot(rej(:,i),rej(:,i+1),'r.');
    xlabel(strcat('q',num2str((i+1)/2)));
    ylabel(strcat('q',num2str((i+1)/2),'_d_o_t'));
end