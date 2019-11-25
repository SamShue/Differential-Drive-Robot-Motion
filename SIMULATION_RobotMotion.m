clc;
clear all;
close all;

robotPose = [0,0,0];
Vr_mps = 0.2;
Vl_mps = 0.2;
dt_s = 0.01; % time between iterations in seconds

for ii = 1: 200
    robotPose = differentialDriveKinematics(Vr_mps, Vl_mps, dt_s, 0.5);
    
    % Render environment
    %======================================================================
    clf;
    hold on;
    xlim([-5 5]); ylim([-5 5]);
    xlabel('meters'); ylabel('meters');
    if(exist('robotPose'))
        drawRobot(robotPose(1), robotPose(2), robotPose(3), 0.25);
    end
    pause(0.001);
    % End render environment
    %----------------------------------------------------------------------
end