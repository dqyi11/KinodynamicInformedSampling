clear; clc; close all;
gibbs = dlmread('test_gibbs.log');
rej = dlmread('test_rej.log');

for i = 1:2:(size(gibbs,2)-1)
    figure; hold on;
    plot(gibbs(:,i),gibbs(:,i+1),'b.');
    plot(rej(:,i),rej(:,i+1),'r.');
    xlabel(strcat('q',num2str((i+1)/2)));
    ylabel(strcat('q',num2str((i+1)/2),'_d_o_t'));
end