data = processAllTopics('/home/jarvis/attitude_test.bag');

%% accelerometers
figure(4); clf; hold on;
[b,a] = butter(6, 0.1);
for i = 1:3
     plot(data.imu.data.time, filtfilt(b,a,data.imu.data.euler(:,i)));
end
hold off