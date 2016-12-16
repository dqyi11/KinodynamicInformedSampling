data = dlmread('test.log');
for i = 1:2:(size(data,2)-1)
    figure; hold on;
    plot(data(:,i),data(:,i+1),'.');
    xlabel(strcat('q',num2str((i+1)/2)));
    ylabel(strcat('q',num2str((i+1)/2),'_d_o_t'));
end