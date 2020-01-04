clear;
loop_times=500;
t=1:loop_times;
right_rate=calculate_P_BCI(loop_times,'result_record.mat');
subplot(2,2,1);
plot(t,right_rate(1,:),t,right_rate(2,:),t,right_rate(3,:),t,right_rate(4,:),t,right_rate(5,:));

right_rate=calculate_P_BCI(loop_times,'onlinedata_dw.mat');
subplot(2,2,2);
plot(t,right_rate(1,:),t,right_rate(2,:),t,right_rate(3,:),t,right_rate(4,:),t,right_rate(5,:));

right_rate=calculate_P_BCI(loop_times,'onlinedata_lxb.mat');
subplot(2,2,3);
plot(t,right_rate(1,:),t,right_rate(2,:),t,right_rate(3,:),t,right_rate(4,:),t,right_rate(5,:));

right_rate=calculate_P_BCI(loop_times,'onlinedata_lyr.mat');
subplot(2,2,4);
plot(t,right_rate(1,:),t,right_rate(2,:),t,right_rate(3,:),t,right_rate(4,:),t,right_rate(5,:));

legend('robot','human','robot&human','robot(without w update)','robot&human(without w update)');