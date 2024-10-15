function q = inverseKinematics(I_r_IE_des, C_IE_des, q_0, tol)

it = 0;
max_it = 100;       % Set the maximum number of iterations. 
lambda = 0.001;     % Damping factor.
alpha = 0.5;        % Update rate

% initial guess
q = q_0;

while (it==0 || (norm(dxe)>tol && it < max_it))
    % Jacobian
    I_J = [jointToPosJac(q); ...
           jointToRotJac(q)];
    
    % Pseudo-inverse
    I_J_pinv = pseudoInverseMat(I_J, lambda);
    
    % End-effector configuration error vector position/rotation error
    I_r_IE = jointToPosition(q);
    dr = I_r_IE_des - I_r_IE; 
    C_IE = jointToRotMat_solution(q);
    C_err = C_IE_des*C_IE';
    dph = rotMatToRotVec_solution(C_err); 
    dxe = [dr; dph];
    
    % Update the generalized coordinates
    q = q + alpha*I_J_pinv*dxe;
    
    it = it+1;
end

end
