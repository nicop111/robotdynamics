function J_P = jointToPosJac(q)
  
  
  %Rotation Axes in 0-frame, reuse jointToRotJac(q) because it contains all Axes
  J_R = jointToRotJac(q);
  
  n1 = J_R(:,1);
  n2 = J_R(:,2);
  n3 = J_R(:,3);
  n4 = J_R(:,4);
  n5 = J_R(:,5);
  n6 = J_R(:,6);


  %R Vectors from 0 to k in 0-frame from Homogenous Transformation matrices
  r1 = jointToTransform01(q)*[0;0;0;1]; 
  r2 = jointToTransform01(q)*jointToTransform12(q)*[0;0;0;1];
  r3 = jointToTransform01(q)*jointToTransform12(q)*jointToTransform23(q)*[0;0;0;1];
  r4 = jointToTransform01(q)*jointToTransform12(q)*jointToTransform23(q)*jointToTransform34(q)*[0;0;0;1];
  r5 = jointToTransform01(q)*jointToTransform12(q)*jointToTransform23(q)*jointToTransform34(q)*jointToTransform45(q)*[0;0;0;1];
  r6 = jointToTransform01(q)*jointToTransform12(q)*jointToTransform23(q)*jointToTransform34(q)*jointToTransform45(q)*jointToTransform56(q)*[0;0;0;1];

  r1=r1(1:3);
  r2=r2(1:3);
  r3=r3(1:3);
  r4=r4(1:3);
  r5=r5(1:3);
  r6=r6(1:3);

  %R Vectors from k to end in 0-frame
  r1E = r6-r1;
  r2E = r6-r2;
  r3E = r6-r3;
  r4E = r6-r4;
  r5E = r6-r5;
  r6E = r6-r6;

  % Compute the translatory jacobian
  J_P = [cross(n1,r1E) cross(n2,r2E) cross(n3,r3E) cross(n4,r4E) cross(n5,r5E) cross(n6,r6E)];
    
end