function T = jointToTI0(q)
  
  r = [0;...
      0;...
      0];
  
  ang = 0;

  rotAx = 'x';

  
  switch rotAx
    case 'x'
      rotation = [1 0 0; 0 cos(ang) -sin(ang); 0 sin(ang) cos(ang)];
    case 'y'
      rotation = [cos(ang) 0 sin(ang); 0 1 0; -sin(ang) 0 cos(ang)];
    case 'z'
      rotation = [cos(ang) -sin(ang) 0; sin(ang) cos(ang) 0; 0 0 1];
    otherwise
      rotation = eye(3);
  end

  T = [rotation r; 0 0 0 1];

end