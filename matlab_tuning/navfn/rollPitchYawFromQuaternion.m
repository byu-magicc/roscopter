function [ phi, theta, psi ] = rollPitchYawFromQuaternion( q )
%rollPitchYawFromQuaternion Return attitude in euler angle form
%   Usage:
%      [ roll, pitch, yaw ] = rollPitchYawFromQuaternion( q )
%

if size(q,2) == 1
    ex = q(1);
    ey = q(2);
    ez = q(3);
    eo = q(4);
else
    ex = q(:,1);
    ey = q(:,2);
    ez = q(:,3);
    eo = q(:,4);
end

phi   = atan2(2.0*eo.*ex + 2.0*ey.*ez, eo.*eo + ez.*ez - ex.*ex - ey.*ey);
theta = asin(2.0*eo.*ey - 2.0*ex.*ez);
psi   = atan2(2.0*eo.*ez + 2.0*ex.*ey, eo.*eo + ex.*ex - ey.*ey - ez.*ez);

end
