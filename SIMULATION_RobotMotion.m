clc;
clear all;
close all;

% Populate wheel velocities vector
v = 0.5;
u = [sin(0:0.001:1.5).*v; cos(0:0.001:1.5).*v];
dt_s = 0.01; % time between iterations in seconds

robotPose = [0,0,0];
trackWidth_m = 0.5;

for ii = 1:length(u(1,:))
    Vr_mps = u(1,ii); Vl_mps = u(2,ii);
    
    v_mps = (Vr_mps + Vl_mps) / 2.0;
    w_radps = (Vr_mps - Vl_mps) / trackWidth_m;

    robotPose = differentialDriveKinematics(v_mps, w_radps, dt_s, 'icr');
    
    % Render environment
    %======================================================================
    clf;
    hold on;
    xlim([-5 5]); ylim([-5 5]);
    xlabel('meters'); ylabel('meters');
    if(exist('robotPose'))
        drawRobot(robotPose(1), robotPose(2), rad2deg(robotPose(3)), 0.25);
    end
    pause(0.001);
    % End render environment
    %----------------------------------------------------------------------
end