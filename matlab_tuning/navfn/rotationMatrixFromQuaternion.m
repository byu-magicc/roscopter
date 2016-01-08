function R = rotationMatrixFromQuaternion(q)
% ROTATIONMATRIXFROMQUATERNION(q)
%    Rotation matrix will rotate which frame a vector is expressed in
%    Uses Hamiltonian quaternion notation
%    qo is last

qx = q(1); 
qy = q(2); 
qz = q(3); 
qo = q(4);

R = [qo^2 + qx^2 - qy^2 - qz^2,         2*qo*qz + 2*qx*qy,         2*qx*qz - 2*qo*qy;
             2*qx*qy - 2*qo*qz, qo^2 - qx^2 + qy^2 - qz^2,         2*qo*qx + 2*qy*qz;
             2*qo*qy + 2*qx*qz,         2*qy*qz - 2*qo*qx, qo^2 - qx^2 - qy^2 + qz^2];
        
end

