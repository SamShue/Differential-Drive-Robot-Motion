function [newPose] = differentialDriveKinematics(pose, v_mps, w_radps, dt_s, model)
%DIFFERENTIALDRIVEKINEMATICS Summary of this function goes here
%   Detailed explanation goes here

% If no model is specified, use the 'simple' model
if(nargin < 4), model = 'linear'; end

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
    % The linear model. This model does not account for curvature in motion
    % and applies motion as a linear move then rotation. While not as 
    % accurate as the ICR model, this model's error is very small when 
    % applied at extremely high sampling rates.
    
    theta = pose(3);
    % create rotation matrix to get velocities in global frame
    R = [cos(theta) 0;
         sin(theta) 0;
         0          1];
    u = [v_mps; w_radps];
    % update pose with global frame velocities
    pose = pose + R*u.*dt_s;
end

newPose = pose;
end

