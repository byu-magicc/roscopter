%data = processTopics({'/relative_state','/vo_out','/naze/CG','/altimeter'},'one/naze.bag');
%data = processTopics({'/relative_state','/vo_out','/naze/CG','/altimeter'},'two/naze.bag');
%data = processTopics({'/relative_state','/vo_out','/naze/CG','/altimeter'},'three/naze.bag');
data = processAllTopics('/home/jarvis/.ros/naze.bag');

%% TRANSLATION
figure(1); clf
% Plot xyz
labels = {'forward','right','down'};
% PLOT X
 subplot(3,1,1); hold on;
 cov_area_X = [data.estimate.time, fliplr(data.estimate.time)];
 cov_area_Y = [2*sqrt(data.pose.covariance(1,:)) + data.relative_state.transform.translation(1,:),...
            fliplr(-2*sqrt(data.relative_state.covariance(1,:)) + data.relative_state.transform.translation(1,:))];
 fill(cov_area_X, cov_area_Y,'k', 'facealpha',.5,'edgecolor','none');
 estimate = plot(data.relative_state.time, data.relative_state.transform.translation(1,:),'-b');
 truth = plot(data.naze.CG.time, data.naze.CG.transform.translation(1,:),'-r');
 ylabel(strcat(labels{1},' (m)'));
 legend([estimate, truth],'estimate','truth', 'Location','northoutside','Orientation','horizontal')
 axis([0,80,-2,4])
 hold off;

% PLOT Y
 subplot(3,1,2); hold on;
 cov_area_X = [data.relative_state.time, fliplr(data.relative_state.time)];
 cov_area_Y = [2*sqrt(data.relative_state.covariance(2,:)) + data.relative_state.transform.translation(2,:),...
            fliplr(-2*sqrt(data.relative_state.covariance(2,:)) + data.relative_state.transform.translation(2,:))];
 fill(cov_area_X, cov_area_Y,'k', 'facealpha',.5,'edgecolor','none');
 estimate = plot(data.relative_state.time, data.relative_state.transform.translation(2,:),'-b');
 truth = plot(data.naze.CG.time, -data.naze.CG.transform.translation(2,:),'-r');
 ylabel(strcat(labels{2},' (m)'));
 axis([0,80,-2,4])
 hold off;

% PLOT Z
 subplot(3,1,3); hold on;
 cov_area_X = [data.relative_state.time, fliplr(data.relative_state.time)];
 cov_area_Y = [2*sqrt(data.relative_state.covariance(3,:)) + data.relative_state.transform.translation(3,:),...
            fliplr(-2*sqrt(data.relative_state.covariance(3,:)) + data.relative_state.transform.translation(3,:))];
 fill(cov_area_X, cov_area_Y,'k', 'facealpha',.5,'edgecolor','none');
 estimate = plot(data.relative_state.time, data.relative_state.transform.translation(3,:),'-b');
 truth = plot(data.naze.CG.time, -data.naze.CG.transform.translation(3,:),'-r');
 ylabel(strcat(labels{3},' (m)'));
 axis([0,80,-2,4])
 hold off;

 xlabel('time (sec)');
% plot(data.altimeter.time, -data.altimeter.range,'c.'); hold on;


%% Velocities
% LPF For mocap data
[b,a] = butter(6, 0.1);
figure(2); clf;
labels = {'u', 'v', 'w'};
for i = 1:3
 subplot(3,1,i); hold on;
 cov_area_X = [data.relative_state.time, fliplr(data.relative_state.time)];
 cov_area_Y = [2*sqrt(data.relative_state.covariance(3+i,:)) + data.relative_state.velocity(i,:),...
            fliplr(-2*sqrt(data.relative_state.covariance(3+i,:)) + data.relative_state.velocity(i,:))];
 fill(cov_area_X, cov_area_Y,'k', 'facealpha',.5,'edgecolor','none');
 estimate = plot(data.relative_state.time, data.relative_state.velocity(i,:),'-b');
 truth = plot(data.transformed_mocap.time, filtfilt(b,a,data.transformed_mocap.velocity(i,:)),'-r');
 ylabel(strcat(labels{i},' (m)'));
 axis([0,80,-2,4])
 hold off;
end


%% ROTATION
% LPF For mocap data
[b,a] = butter(6, 0.2);

figure(3); clf;
% Plot euler
labels = {'roll','pitch','yaw'};
for i = 1:2 % roll and pitch
 subplot(3,1,i); hold on;
 cov_area_X = [data.relative_state.time, fliplr(data.relative_state.time)];
 cov_area_Y = [2*sqrt(data.relative_state.covariance(6+i,:)) + data.relative_state.transform.euler(:,i)',...
            fliplr(-2*sqrt(data.relative_state.covariance(6+i,:)) + data.relative_state.transform.euler(:,i)')];
 fill(cov_area_X, cov_area_Y,'k', 'facealpha',.5,'edgecolor','none');
 estimate = plot(data.relative_state.time', data.relative_state.transform.euler(:,i),'-b');
 truth = plot(data.transformed_mocap.time', filtfilt(b,a,data.transformed_mocap.transform.euler(:,i)),'-r');
 ylabel(strcat(labels{i},' (deg)'));
 %legend('Truth','Estimates')
end
% yaw
i = 3;
subplot(3,1,i); hold on;
cov_area_X = [data.relative_state.time, fliplr(data.relative_state.time)];
cov_area_Y = [2*sqrt(data.relative_state.covariance(6+i,:)) + (data.relative_state.transform.euler(:,i)-data.transformed_mocap.transform.euler(1,i))',...
        fliplr(-2*sqrt(data.relative_state.covariance(6+i,:)) + (data.relative_state.transform.euler(:,i)-data.transformed_mocap.transform.euler(1,i))')];
fill(cov_area_X, cov_area_Y,'k', 'facealpha',.5,'edgecolor','none');
estimate = plot(data.relative_state.time', data.relative_state.transform.euler(:,i)-data.transformed_mocap.transform.euler(1,i),'-b');
truth = plot(data.transformed_mocap.time', filtfilt(b,a,data.transformed_mocap.transform.euler(:,i)),'-r');
ylabel(strcat(labels{i},' (deg)'));
%legend('Truth','Estimates')
xlabel('time (sec)');
suptitle('Euler')

%% accelerometers
figure(4); clf; hold on;
[b,a] = butter(6, 0.1);
for i = 1:3
     plot(data.imu.data.time, filtfilt(b,a,data.imu.data.euler(:,i)));
end
hold off
