function data = processTopics(topics,bagfile,t0)
%clear rosbag_wrapper;
%clear ros.Bag;

  if nargin < 3
    t0 = -1;
  end

addpath('./matlab_rosbag-0.4.1-linux64')
addpath('./navfn')

bag = ros.Bag.load(bagfile);
for topic = topics
    ind = find(ismember(bag.topics,topic),1);
    if isempty(ind)
        fprintf('Could not find topic: %s\n',topic{:});
        continue;
    end
    fprintf('   Processing topic: %s\n',topic{:});
    type = bag.topicType(topic{:});    
    [msgs, meta] = bag.readAll(topic);
    
    a = [msgs{:,:}];
    c = [meta{:,:}];
    d = [c.time];
    if t0 < 0
       t0 = d(1).time - 0.1;
    end
    clear struct
    switch type{:}
        case 'relative_nav/FilterState'
            b = [a.transform];
            struct.transform.translation = [b.translation];
            struct.transform.rotation = [b.rotation];
            [r,p,y] = rollPitchYawFromQuaternion(struct.transform.rotation.');
            struct.transform.euler = [r,p,y]*180/pi;
            struct.covariance = [a.covariance];
            struct.velocity = [a.velocity];     
            struct.node_id = [a.node_id];
            struct.time = [d.time] - t0;
         case 'sensor_msgs/NavSatFix'
            struct.latitude = [a.latitude];
            struct.longitude = [a.longitude];
            cov = [a.position_covariance];
            struct.hAcc = cov(1,:);
            struct.time = [d.time] - t0;
        case 'geometry_msgs/Transform'
            struct.transform.translation = [a.translation];
            struct.transform.rotation = [a.rotation];
            [r,p,y] = rollPitchYawFromQuaternion(struct.transform.rotation.');
            struct.transform.euler = [r,p,y]*180/pi;
            struct.time = [d.time] - t0;
        case 'geometry_msgs/TransformStamped'
            b = [a.transform];
            struct.transform.translation = [b.translation];
            struct.transform.rotation = [b.rotation];
            [r,p,y] = rollPitchYawFromQuaternion(struct.transform.rotation.');
            struct.transform.euler = [r,p,y]*180/pi;
            struct.time = [d.time] - t0;
        case 'relative_nav/DesiredState'
            struct.pose = [a.pose];
            struct.velocity = [a.velocity];
            struct.acceleration = [a.acceleration];
            struct.node_id = [a.node_id];
            
            struct.time = [d.time] - t0;
        case 'relative_nav/Snapshot'
            struct.state = [a.state];
            struct.covariance = [a.covariance_diagonal];
            struct.node_id = [a.node_id];
            struct.time = [d.time] - t0; % This could also use the header time
        case 'geometry_msgs/PoseStamped'
            b = [a.pose];
            struct.pose.position = [b.position];
            struct.pose.orientation = [b.orientation];
            struct.time = [d.time] - t0;
        case 'sensor_msgs/Range'
            struct.range = [a.range];
            struct.time = [d.time] -t0; % This could also use the header time
        case 'relative_nav/VOUpdate'
            b = [a.transform];
            struct.current_keyframe_id = [a.current_keyframe_id];
            struct.new_keyframe = [a.new_keyframe];
            struct.valid_transformation = [a.valid_transformation];
            struct.transform.translation = [b.translation];
            struct.transform.rotation = [b.rotation];
            [r,p,y] = rollPitchYawFromQuaternion(struct.transform.rotation.');
            struct.transform.euler = [r,p,y]*180/pi;
            struct.time = [d.time] -t0; % This could also use the header time
        case 'relative_nav/Command'
            struct.commands = [a];
            struct.time = [d.time] -t0; % This could also use the header time
        case 'evart_bridge/transform_plus'
            b = [a.transform];
            struct.transform.translation = [b.translation];
            struct.transform.rotation = [b.rotation];
            struct.transform.euler = rollPitchYawFromQuaternion(struct.transform.rotation.')*180/pi;
            struct.euler = [a.euler];
            struct.velocity = [a.velocity];
            struct.time = [d.time] -t0; % This could also use the header time
        case 'relative_nav/Edge'
            b = [a.transform];
            struct.transform.translation = [b.translation];
            struct.transform.rotation = [b.rotation];
            struct.transform.euler = rollPitchYawFromQuaternion(struct.transform.rotation.')*180/pi;
            struct.from_node_id = [a.from_node_id];
            struct.to_node_id = [a.to_node_id];
            struct.covariance = [a.covariance];
            struct.time = [d.time] -t0; % This could also use the header time    
        case 'nav_msgs/Odometry'
            b = [a.pose];
            c = [b.pose];
            e = [a.twist];
            f = [e.twist];
            pos_cov = [b.covariance];
            vel_cov = [e.covariance];
            struct.pose.position = [c.position];
            struct.pose.orientation = [c.orientation];
            [r,p,y] = rollPitchYawFromQuaternion(struct.pose.orientation.');
            struct.pose.euler = [r,p,y]*180/pi;
            struct.pose.covariance = [pos_cov(1,:);...
                                      pos_cov(8,:);...
                                      pos_cov(15,:);...
                                      pos_cov(22,:);...
                                      pos_cov(29,:);...
                                      pos_cov(36,:)];
            struct.vel.linear = [f.linear];
            struct.vel.angular = [f.angular];
            struct.vel.covariance = [vel_cov(1,:);...
                                     vel_cov(8,:);...
                                     vel_cov(15,:);...
                                     vel_cov(22,:);...
                                     vel_cov(29,:);...
                                     vel_cov(36,:)];
            struct.time = [d.time] -t0; % This could also use the header time    
        case 'geometry_msgs/Point'
            struct.point = a;
            struct.time = [d.time] -t0; % This could also use the header time
        case 'ublox_msgs/NavPOSLLH'
            struct.lon = double([a.lon])/1e7;
            struct.lat = double([a.lat])/1e7;
            struct.hAcc = [a.hAcc];
            [struct.x,struct.y,struct.zone] = deg2utm(struct.lat,struct.lon);
        case 'sensor_msgs/NavSatFix'
            struct.lon = [a.longitude];
            struct.lat = [a.latitude];
            cov = [a.position_covariance];
            struct.cov = cov(1,:);
            [struct.x,struct.y,struct.zone] = deg2utm(struct.lat,struct.lon);
            struct.time = [d.time] -t0; % This could also use the header time    
        case 'sensor_msgs/Imu'
            struct.acc = [a.linear_acceleration];
            struct.gyro = [a.angular_velocity];
            struct.orientation = [a.orientation];
            [r,p,y] = rollPitchYawFromQuaternion(struct.orientation');
            struct.euler = [r,p,y]*180/pi;
            struct.time = [d.time] -t0; % This could also use the header time    
        case 'sensor_msgs/LaserScan'
            struct.angle_min = [a.angle_min];
            struct.angle_max = [a.angle_max];
            struct.angle_increment = [a.angle_increment];
            struct.time_increment = [a.time_increment];
            struct.range_min = [a.range_min];
            struct.range_max = [a.range_max];
            struct.ranges = [a.ranges];
            struct.intensities = [a.intensities];            
        case 'visualization_msgs/Marker'
            number = 1;
            for i = a(end:-1:1)
               if(size(i.points,2) > 2)
                   if(number == 1)
                       struct.opt.points = i.points;
                       number = 2;
                   elseif(number == 2)
                       struct.unopt.points = i.points;
                       break
                   else
                    break
                   end
               end
            end            
        otherwise
            fprintf('     Type: %s not yet supported!\n',type{:});
            continue
    end
    
    % Split topic name into sections
    topic_parts = strsplit(topic{1},'/');
    topic_parts(cellfun('isempty', topic_parts)) = [];
    switch size(topic_parts,2)
        case 1
            data.(topic_parts{1}) = struct;
        case 2
            data.(topic_parts{1}).(topic_parts{2}) = struct;
        case 3
            data.(topic_parts{1}).(topic_parts{2}).(topic_parts{3}) = struct;
        case 4
            data.(topic_parts{1}).(topic_parts{2}).(topic_parts{3}).(topic_parts{4}) = struct;
        case 5
            data.(topic_parts{1}).(topic_parts{2}).(topic_parts{3}).(topic_parts{4}).(topic_parts{5}) = struct;
        otherwise
            fprintf('Too long');
    end     
end


end
