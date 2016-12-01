x = -1:0.05:1;
y = sigmf(x,[20 0]);
plot(x,y)
xlabel('sigmf, P = [2 4]')
ylim([-0.05 1.05])