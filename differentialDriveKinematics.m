function [robotPose] = differentialDriveKinematics(Vr_mps, Vl_mps, dt_s, trackWidth_m)
%DIFFERENTIALDRIVEKINEMATICS Summary of this function goes here
%   Detailed explanation goes here
persistent pose;
if isempty(pose)
    pose = [0,0,0];
end

R = (Vr_mps + Vl_mps)/(Vr_mps - Vl_mps);
w_radps = (Vr_mps - Vl_mps) / trackWidth_m;

if(isinf(R))
    ICC = [0.0, 0.0];
else
    ICC = [pose(1) - R*sin(pose(3)), pose(2) + R*cos(pose(3))];
end

kinematics = [cos(w_radps*dt_s), -1.0*sin(w_radps*dt_s), 0;
              sin(w_radps*dt_s), cos(w_radps*dt_s), 0;
              0, 0, 1];
          
pose = kinematics * [pose(1) - ICC(1); pose(2) - ICC(2); pose(3)] + [ICC(1); ICC(2); dt_s*w_radps];
robotPose = pose;
end

