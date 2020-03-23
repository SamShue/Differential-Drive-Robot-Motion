function [robotPose] = differentialDriveKinematics(v_mps, w_radps, dt_s, model)
%DIFFERENTIALDRIVEKINEMATICS Summary of this function goes here
%   Detailed explanation goes here

% Initialize pose variable
persistent pose;
if isempty(pose)
    pose = [0,0,0];
end

% If no model is specified, use the 'simple' model
if(nargin < 3), model = 'linear'; end

if(strcmp(model, 'icr'))
    % compute radius of curvature
    r = abs(v_mps/w_radps);
    
    
    theta = pose(3);
    % compute sin and cos at initial pose
    s = sin(theta);
    c = cos(theta);
    % copute sin and cos at final pose
    s_th = sin(theta + w_radps*dt_s);
    c_th = cos(theta + w_radps*dt_s);

    if(w_radps < 0.05)
        % if the robot is moving straight, then that corresponds to a 
        % circle with an infinite radius. Here we must exclude the r
        % variable to account for that.
        pose(1) = pose(1) + (v_mps*c_th)*dt_s;
        pose(2) = pose(2) + (v_mps*s_th)*dt_s;
        pose(3) = pose(3) + w_radps*dt_s;
    else
        % robot is moving with some curavature
        pose(1) = pose(1) + (-r*s)+(r*s_th);
        pose(2) = pose(2) +( r*c)-(r*c_th);
        pose(3) = pose(3) + w_radps*dt_s;
    end
else
    
end

robotPose = pose;
end

