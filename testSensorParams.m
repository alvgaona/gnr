function [avg,cov] = testSensorParams(nMeasures,sensorName,plotBool)
%TESTSENSORPARAMS Tests n sensor measures, and calculates average and
%covariance of the measures
%   Detailed explanation goes here
    if(nargin==2)plotBool=false;end
    center_measures=zeros(1,nMeasures);
    for i = 1:nMeasures
        meas=apoloGetLaserData(sensorName);
        siz=size(meas);
        if siz(2) > 181 %LMS100
            %center_measures(i) = meas(round(539/2));
        else %LMS200
            meas = meas(1:180); %measure 181 is always 0...
        end
        center_measures(i) = meas(round(size(meas,2)/2)); %get the forward facing laser
        %pause(0.5);
        apoloUpdate();
    end
    avg = mean(center_measures);
    if plotBool
        figure("Name","Calibration results");
        subplot(1,2,1),title("Measures through time"),plot(center_measures);
        subplot(1,2,2),title("Measure distribution"),histogram(center_measures);
        xline(avg,'--r',{'Average','= '+string(avg)});
    end
    cov = std(center_measures);
end