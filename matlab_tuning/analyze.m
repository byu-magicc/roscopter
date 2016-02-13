%data = processTopics({'/relative_state','/vo_out','/naze/CG','/altimeter'},'one/naze.bag');
%data = processTopics({'/relative_state','/vo_out','/naze/CG','/altimeter'},'two/naze.bag');
%data = processTopics({'/relative_state','/vo_out','/naze/CG','/altimeter'},'three/naze.bag');
data = processAllTopics('~/.ros/naze.bag');
t0 = 0;
tf = 35;

%% TRANSLATION
figure(1); clf
% Plot xyz
labels = {'forward','right','down'};
% PLOT X
 subplot(3,1,1); hold on;
 cov_area_X = [data.estimate.time, fliplr(data.estimate.time)];
 cov_area_Y = [6*sqrt(data.estimate.pose.covariance(1,:)) + data.estimate.pose.position(1,:),...
            fliplr(-6*sqrt(data.estimate.pose.covariance(1,:)) + data.estimate.pose.position(1,:))];
 fill(cov_area_X, cov_area_Y,'k', 'facealpha',.5,'edgecolor','none');
 estimate = plot(data.estimate.time, data.estimate.pose.position(1,:),'-b');
 truth = plot(data.naze.CG.time, data.naze.CG.transform.translation(1,:),'-r');
 ylabel(strcat(labels{1},' (m)'));
 legend([estimate, truth],'estimate','truth', 'Location','northoutside','Orientation','horizontal')
 axis([t0,tf,-2,4])
 hold off;

% PLOT Y
 subplot(3,1,2); hold on;
 cov_area_X = [data.estimate.time, fliplr(data.estimate.time)];
 cov_area_Y = [4*sqrt(data.estimate.pose.covariance(2,:)) + data.estimate.pose.position(2,:),...
            fliplr(-4*sqrt(data.estimate.pose.covariance(2,:)) + data.estimate.pose.position(2,:))];
 fill(cov_area_X, cov_area_Y,'k', 'facealpha',.5,'edgecolor','none');
 estimate = plot(data.estimate.time, data.estimate.pose.position(2,:),'-b');
 truth = plot(data.naze.CG.time, -data.naze.CG.transform.translation(2,:),'-r');
 ylabel(strcat(labels{2},' (m)'));
 axis([t0,tf,-2,4])
 hold off;

% PLOT Z
 subplot(3,1,3); hold on;
 cov_area_X = [data.estimate.time, fliplr(data.estimate.time)];
 cov_area_Y = [4*sqrt(data.estimate.pose.covariance(3,:)) + data.estimate.pose.position(3,:),...
            fliplr(-4*sqrt(data.estimate.pose.covariance(3,:)) + data.estimate.pose.position(3,:))];
 fill(cov_area_X, cov_area_Y,'k', 'facealpha',.5,'edgecolor','none');
 estimate = plot(data.estimate.time, data.estimate.pose.position(3,:),'-b');
 truth = plot(data.naze.CG.time, -data.naze.CG.transform.translation(3,:),'-r');
 ylabel(strcat(labels{3},' (m)'));
 axis([t0,tf,-2,4])
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
 cov_area_X = [data.estimate.time, fliplr(data.estimate.time)];
 cov_area_Y = [2*sqrt(data.estimate.vel.covariance(3+i,:)) + data.estimate.vel.linear(i,:),...
            fliplr(-2*sqrt(data.estimate.vel.covariance(3+i,:)) + data.estimate.vel.linear(i,:))];
 fill(cov_area_X, cov_area_Y,'k', 'facealpha',.5,'edgecolor','none');
 estimate = plot(data.estimate.time, data.estimate.vel.linear(i,:),'-b');
 truth = plot(data.transformed_mocap.time, filtfilt(b,a,data.transformed_mocap.velocity(i,:)),'-r');
 ylabel(strcat(labels{i},' (m)'));
 axis([t0,tf,-2,4])
 hold off;
 if i == 1
   legend([estimate, truth],'estimate','truth', 'Location','northoutside','Orientation','horizontal')
 end
end


%% ROTATION
% LPF For mocap data
[b,a] = butter(6, 0.9);

figure(3); clf;
% Plot euler
labels = {'roll','pitch','yaw'};
for i = 1:2 % roll and pitch
 subplot(3,1,i); hold on;
 cov_area_X = [data.estimate.time, fliplr(data.estimate.time)];
 cov_area_Y = [100*sqrt(data.estimate.pose.covariance(3+i,:)) + data.estimate.pose.euler(:,i)',...
            fliplr(-100*sqrt(data.estimate.pose.covariance(3+i,:)) + data.estimate.pose.euler(:,i)')];
 fill(cov_area_X, cov_area_Y,'k', 'facealpha',.5,'edgecolor','none');
 estimate = plot(data.estimate.time', data.estimate.pose.euler(:,i),'-b');
 truth = plot(data.transformed_mocap.time', filtfilt(b,a,data.transformed_mocap.transform.euler(:,i)),'-r');
 ylabel(strcat(labels{i},' (deg)'));
 axis([t0,tf,-30,30])
 if i == 1
   legend([estimate, truth],'estimate','truth', 'Location','northoutside','Orientation','horizontal')
 end
 %legend('Truth','Estimates')
end
% yaw
i = 3;
subplot(3,1,i); hold on;
cov_area_X = [data.estimate.time, fliplr(data.estimate.time)];
cov_area_Y = [100*sqrt(data.estimate.pose.covariance(3+i,:)) + (data.estimate.pose.euler(:,i)-data.estimate.pose.euler(1,i))',...
        fliplr(-100*sqrt(data.estimate.pose.covariance(3+i,:)) + (data.estimate.pose.euler(:,i)-data.estimate.pose.euler(1,i))')];
fill(cov_area_X, cov_area_Y,'k', 'facealpha',.5,'edgecolor','none');
estimate = plot(data.estimate.time', data.estimate.pose.euler(:,i)-data.estimate.pose.euler(1,i),'-b');
truth = plot(data.transformed_mocap.time', filtfilt(b,a,data.transformed_mocap.transform.euler(:,i))-mean(data.transformed_mocap.transform.euler(1:10,3)),'-r');
ylabel(strcat(labels{i},' (deg)'));
xlabel('time (sec)');
axis([t0,tf,-30,30])

%% accelerometer biases
figure(4); clf; hold on;
[b,a] = butter(6, 0.1);
for i = 1:3
    cov_area_X = [data.estimate.bias.time, fliplr(data.estimate.bias.time)];
    plot(data.estimate.bias.time, data.estimate.bias.acc(i,:));
end
title('accel biases');
hold off


%% accelerometer readings
figure(5); clf; hold on;
[b,a] = butter(6, 0.1);
for i = 1:3
    plot(data.imu.data.time, data.imu.data.acc(i,:));
end
title('accel readings');
hold off;
    
