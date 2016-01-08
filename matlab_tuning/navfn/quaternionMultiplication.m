function [ pq ] = quaternionMultiplication( p,q )
%QUATERNIONMULTIPLICATION Summary of this function goes here
%   Detailed explanation goes here
% qx qy qz qo ordering

px = p(1);
py = p(2);
pz = p(3);
po = p(4);

pq = [po -pz  py  px;
      pz  po -px  py;
     -py  px  po  pz;
     -px -py -pz  po]*q;
end

