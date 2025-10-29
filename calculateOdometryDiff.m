function [Ad,AB] = calculateOdometryDiff(robot,prevOdometry)
% CALCULATEODOMETRYDIFF - Calculates the delta [Δd Δβ] in the reported 
% odometry from the Apolo simulator.
% 
% The velocity command and `apoloUpdate()` should be run between the 
% measure of prevOdometry and this function.
% 
% Input Arguments:
%  - robot (char) -
%    Char string with the name of the robot in the simulator
%  - prevOdometry (double) -
%    Measured position based on odometry in the previous step [x y theta]
% 
% Output Arguments:
%  - Ad (double) - distance delta (Δd) or difference between steps
%  - AB (double) - angle delta (Δβ) or difference between steps
% 
% Usage:
%  Example 1 - Normal use::
%
%    meas=apoloGetOdometry(robotName);
%    apoloMoveMRobot(robotName,[linearVelCmd,angularVelCmd],Atime);
%    apoloUpdate();
%    [d,B]=calculateOdometryDiff(robotName,meas);
% See also APOLOUPDATE APOLOMOVEMROBOT
    arguments
        robot char
        prevOdometry (1,3) double
    end

    newOdometry=apoloGetOdometry(robot);
    Ax=newOdometry(1)-prevOdometry(1);
    Ay=newOdometry(2)-prevOdometry(2);
    Ad=norm([Ax,Ay]);%sqrt(Ax*Ax+Ay*Ay) with abs() on Ax and Ay
    AB=newOdometry(3)-prevOdometry(3);
end