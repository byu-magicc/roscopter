%data = processTopics({'/relative_state','/vo_out','/naze/CG','/altimeter'},'one/naze.bag');
%data = processTopics({'/relative_state','/vo_out','/naze/CG','/altimeter'},'two/naze.bag');
%data = processTopics({'/relative_state','/vo_out','/naze/CG','/altimeter'},'three/naze.bag');
data = processAllTopics('/home/iman/.ros/naze.bag');

%% TRANSLATION
figure(1); clf
% Plot xyz
labels = {'forward','right','down'};
% PLOT X
 subplot(3,1,1); hold on;
 cov_area_X = [data.estimate.time, fliplr(data.estimate.time)];
 cov_area_Y = [2*sqrt(data.estimate.covariance(1,:)) + data.estimate.transform.translation(1,:),...
            fliplr(-2*sqrt(data.estimate.covariance(1,:)) + data.estimate.transform.translation(1,:))];
 fill(cov_area_X, cov_area_Y,'k', 'facealpha',.5,'edgecolor','none');
 estimate = plot(data.estimate.time, data.estimate.transform.translation(1,:),'-b');
 truth = plot(data.naze.CG.time, data.naze.CG.transform.translation(1,:),'-r');
 ylabel(strcat(labels{1},' (m)'));
 legend([estimate, truth],'estimate','truth', 'Location','northoutside','Orientation','horizontal')
 axis([0,20,-2,4])
 hold off;

% PLOT Y
 subplot(3,1,2); hold on;
 cov_area_X = [data.estimate.time, fliplr(data.estimate.time)];
 cov_area_Y = [2*sqrt(data.estimate.covariance(2,:)) + data.estimate.transform.translation(2,:),...
            fliplr(-2*sqrt(data.estimate.covariance(2,:)) + data.estimate.transform.translation(2,:))];
 fill(cov_area_X, cov_area_Y,'k', 'facealpha',.5,'edgecolor','none');
 estimate = plot(data.estimate.time, data.estimate.transform.translation(2,:),'-b');
 truth = plot(data.naze.CG.time, -data.naze.CG.transform.translation(2,:),'-r');
 ylabel(strcat(labels{2},' (m)'));
 axis([0,20,-2,4])
 hold off;

% PLOT Z
 subplot(3,1,3); hold on;
 cov_area_X = [data.estimate.time, fliplr(data.estimate.time)];
 cov_area_Y = [2*sqrt(data.estimate.covariance(3,:)) + data.estimate.transform.translation(3,:),...
            fliplr(-2*sqrt(data.estimate.covariance(3,:)) + data.estimate.transform.translation(3,:))];
 fill(cov_area_X, cov_area_Y,'k', 'facealpha',.5,'edgecolor','none');
 estimate = plot(data.estimate.time, data.estimate.transform.translation(3,:),'-b');
 truth = plot(data.naze.CG.time, -data.naze.CG.transform.translation(3,:),'-r');
 ylabel(strcat(labels{3},' (m)'));
 axis([0,20,-2,4])
 hold off;

 xlabel('time (sec)');
% plot(data.altimeter.time, -data.altimeter.range,'c.'); hold on;

%% Velocities
figure(2); clf;
labels = {'u', 'v', 'w'};
for i = 1:3
 subplot(3,1,i); hold on;
 cov_area_X = [data.estimate.time, fliplr(data.estimate.time)];
 cov_area_Y = [2*sqrt(data.estimate.covariance(3+i,:)) + data.estimate.velocity(i,:),...
            fliplr(-2*sqrt(data.estimate.covariance(3+i,:)) + data.estimate.velocity(i,:))];
 fill(cov_area_X, cov_area_Y,'k', 'facealpha',.5,'edgecolor','none');
 estimate = plot(data.estimate.time, data.estimate.velocity(i,:),'-b');
 ylabel(strcat(labels{1},' (m)'));
 axis([0,20,-2,4])
 hold off;
end


%% ROTATION

figure(3); clf
% Plot euler
labels = {'roll','pitch','yaw'};
for i = 1:3
 subplot(3,1,i); hold on;
 cov_area_X = [data.estimate.time, fliplr(data.estimate.time)];
 cov_area_Y = [20*sqrt(data.estimate.covariance(6+i,:)) + data.estimate.transform.euler(:,i)',...
            fliplr(-20*sqrt(data.estimate.covariance(6+i,:)) + data.estimate.transform.euler(:,i)')];
 fill(cov_area_X, cov_area_Y,'k', 'facealpha',.5,'edgecolor','none');
 estimate = plot(data.estimate.time', data.estimate.transform.euler(:,i),'b');
 ylabel(strcat(labels{i},' (deg)'));
 %legend('Truth','Estimates')
end
xlabel('time (sec)');
suptitle('Euler')