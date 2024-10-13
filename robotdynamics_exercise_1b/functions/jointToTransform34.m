function T34 = jointToTransform34(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 4 to frame 3. T_34
  
  phi_deg = (360/2/pi)*q(4);
  
  translation = [.134; 0; .07];
  
  rotation = rotx(phi_deg);

  T34 = [rotation translation;
         0 0 0 1];
end

