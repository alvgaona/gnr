clear,clc;
%simulation parameters
Atime=0.02; %seconds
linearVelCmd=0;%m/s
angularVelCmd=0.1;%rad/s
robotName='Marvin';
nMeasures=100;
measures=zeros(2,nMeasures);
apoloResetOdometry(robotName);

for i = 1:nMeasures
    meas=apoloGetOdometry(robotName);
    apoloMoveMRobot(robotName,[linearVelCmd,angularVelCmd],Atime);
    apoloUpdate();
    [d,B]=calculateOdometryDiff(robotName,meas);
    measures(1,i)=d;measures(2,i)=B;
end
avgd = mean(measures(1,:));
covd = std(measures(1,:));
avgB = mean(measures(2,:));
covB = std(measures(2,:));

%Atime=0.02;linearVelCmd=0.1;angularVelCmd=0;%rad/s->avgd=0.002,avgB=covB=0,covd=9.5003e-05
%Atime=0.02;linearVelCmd=0;angularVelCmd=0.1;%rad/s->avgd=covd=0,avgB=0.002,covB=3.9080e-05
