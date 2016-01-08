% plot_estimates.m

time = [0 500];

figure(1); clf
% Plot xyz
labels = {'forward','right','down'};
trim = data.relative_state.time < time(end) & data.relative_state.time > time(1);
new_node_mask = [diff(data.relative_state.node_id) == 1 0] & trim;
for i = 1:3
 subplot(3,1,i)
 plot(data.relative_state.time(trim), data.relative_state.transform.translation(i,trim),'b.'); hold on;
 plot(data.relative_state.time(new_node_mask), data.relative_state.transform.translation(i,new_node_mask),'ro'); 
 ylabel(strcat(labels{i},' (m)'));
 xlabel('time (sec)');
end
suptitle('Position')
%plot(data.mavros.distance_sensor.time, -data.mavros.distance_sensor.range,'co');

figure(2); clf
% Plot euler
labels = {'roll','pitch','yaw'};
for i = 1:3
 subplot(3,1,i)
 plot(data.relative_state.time(trim), data.relative_state.transform.euler(i,trim),'b.'); hold on;
 plot(data.relative_state.time(new_node_mask), data.relative_state.transform.euler(i,new_node_mask),'ro');
 ylabel(strcat(labels{i},' (deg)'));
 xlabel('time (sec)');
 %legend('Truth','Estimates')
end
suptitle('Euler')

figure(3); clf
% Plot uvw
labels = {'u','v','w'};
for i = 1:3
 subplot(3,1,i)
 plot(data.relative_state.time(trim), data.relative_state.velocity(i,trim),'b.'); hold on;
 plot(data.relative_state.time(new_node_mask), data.relative_state.velocity(i,new_node_mask),'ro');
 ylabel(strcat(labels{i},' (deg)'));
 xlabel('time (sec)');
end
suptitle('Velocity')

figure(4); clf
% Plot Beta
labels = {'Bx','By','Bz'};
for i = 1:3
 subplot(3,1,i)
 plot(data.relative_state.snapshot.time(trim), data.relative_state.snapshot.state(10+i,trim),'b.');
 ylabel(strcat(labels{i},''));
 xlabel('time (sec)');
end
suptitle('Gyro Biases')

figure(5); clf
% Plot Beta
labels = {'Ax','Ay','mu'};
for i = 1:3
 subplot(3,1,i)
 plot(data.relative_state.snapshot.time(trim), data.relative_state.snapshot.state(13+i,trim),'b.');
 ylabel(strcat(labels{i},''));
 xlabel('time (sec)');
end
suptitle('Accel Biases')

trim_vo = data.sensors.vo_cmu.time < time(end) & data.sensors.vo_cmu.time > time(1);
figure(6); clf
% Plot VO
labels = {'Tx','Ty','Tz'};
vo = [3 1 2]; % remap
for i = 1:3
 subplot(3,1,i)
 plot(data.sensors.vo_cmu.time(trim_vo), -data.sensors.vo_cmu.transform.translation(vo(i),trim_vo),'r.'); hold on
 plot(data.relative_state.time(trim), data.relative_state.transform.translation(i,trim),'b.'); hold on;
 ylabel(strcat(labels{i},''));
 xlabel('time (sec)');
end
suptitle('VOT')

figure(7); clf
% Plot desired state
for i = 1:2
 subplot(2,1,i)
 plot(data.desired_state.time, data.desired_state.velocity(i,:),'r.'); hold on;
 plot(data.relative_state.time(trim), data.relative_state.velocity(i,trim),'b.'); hold on;
 
 ylabel(strcat(labels{i},''));
 xlabel('time (sec)');
end
suptitle('Desired Velocity')