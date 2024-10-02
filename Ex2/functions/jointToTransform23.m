function T23 = jointToTransform23(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 3 to frame 2. T_23
  
  phi_deg = (360/2/pi)*q(3);
  
  translation = [0;0;.27];
  
  rotation = roty(phi_deg);

  T23 = [rotation translation;
         0 0 0 1];
end
