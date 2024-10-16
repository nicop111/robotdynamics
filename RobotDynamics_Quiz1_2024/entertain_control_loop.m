% This script will allow you to see a visualization of the robot
% tracking the entertain (dance) reference trajectory
% (If visualization gets stuck, press Ctrl + C in the Command Window)

close all;
clear params;
init_workspace; % also initializes params

% Set to 0 to test your own solution, set to 1 to see how the solution should look like
use_solution = 0;

%% Control Loop
track_entertain_trajectory(use_solution, params);