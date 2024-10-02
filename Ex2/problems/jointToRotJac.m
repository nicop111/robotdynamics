function J_R = jointToRotJac(q)
  
  %Rotations
  C01 = rotz((360/2/pi) * q(1));
  C12 = roty((360/2/pi) * q(2));
  C23 = roty((360/2/pi) * q(3));
  C34 = rotx((360/2/pi) * q(4));
  C45 = roty((360/2/pi) * q(5));
  C56 = rotx((360/2/pi) * q(6));

  %Rotation axes in 0 frame
  n1 = [0;0;1];
  n2 = C01 * [0;1;0];
  n3 = C01 * C12 * [0;1;0];
  n4 = C01 * C12 * C23 * [1;0;0];
  n5 = C01 * C12 * C23 * C34 * [0;1;0];
  n6 = C01 * C12 * C23 * C34 * C45 * [1;0;0];


  J_R = [n1 n2 n3 n4 n5 n6];

end


% %Rotation Vactors in joint frames
%   n1 = [0;0;1;1];
%   n2 = [0;1;0;1];
%   n3 = [0;1;0;1];
%   n4 = [1;0;0;1];
%   n5 = [0;1;0;1];
%   n6 = [1;0;0;1];
% 
%   %transform into frame 5
%   n6 = jointToTransform56(q)*n6;
% 
%   %transform into frame 4
%   n6 = jointToTransform45(q)*n6;
%   n5 = jointToTransform45(q)*n5;
% 
%   %transform into frame 3
%   n6 = jointToTransform34(q)*n6;
%   n5 = jointToTransform34(q)*n5;
%   n4 = jointToTransform34(q)*n4;
% 
%   %transform into frame 2
%   n6 = jointToTransform23(q)*n6;
%   n5 = jointToTransform23(q)*n5;
%   n4 = jointToTransform23(q)*n4;
%   n3 = jointToTransform23(q)*n3;
% 
%   %transform into frame 1
%   n6 = jointToTransform12(q)*n6;
%   n5 = jointToTransform12(q)*n5;
%   n4 = jointToTransform12(q)*n4;
%   n3 = jointToTransform12(q)*n3;
%   n2 = jointToTransform12(q)*n2;
% 
%   %transform into frame 0
%   n6 = jointToTransform01(q)*n6;
%   n5 = jointToTransform01(q)*n5;
%   n4 = jointToTransform01(q)*n4;
%   n3 = jointToTransform01(q)*n3;
%   n2 = jointToTransform01(q)*n2;
%   n1 = jointToTransform01(q)*n1;
% 
%   %cut & normalize vectors
%   n6 = n6(1:3)/norm(n6(1:3));
%   n5 = n5(1:3)/norm(n5(1:3));
%   n4 = n4(1:3)/norm(n4(1:3));
%   n3 = n3(1:3)/norm(n3(1:3));
%   n2 = n2(1:3)/norm(n2(1:3));
%   n1 = n1(1:3)/norm(n1(1:3));