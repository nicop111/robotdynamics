function T56 = jointToTransform56(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 6 to frame 5. T_56
  
  phi_deg = (360/2/pi)*q(6);
  
  translation = [.072; 0; 0];
  
  rotation = rotx(phi_deg);

  T56 = [rotation translation;
         0 0 0 1];
end
