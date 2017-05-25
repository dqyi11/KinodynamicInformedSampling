datahmc = datatesthmc;
datahrs = datatesthrs;
datahitnrun = datatesthitnrun;
datarej = datatestrej;

figure;
for i= 1:1:12
    subplot(4,3,i);
    boxplot([datahmc(:,i), datahrs(:,i), datahitnrun(:,i), datarej(:,i)],'labels',{'HMC','HRS','Hit&Run','Rejection'})
    title(sprintf('Dimensions %d', i))
end


