function T45 = jointToTransform45(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 5 to frame 4. T_45
  
  phi_deg = (360/2/pi)*q(5);
  
  translation = [(.374-.134-.072); 0; 0];
  
  rotation = roty(phi_deg);

  T45 = [rotation translation;
         0 0 0 1];
end

