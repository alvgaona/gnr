nMeasures=100;sensorName='LMS100';landmark=true;
%TESTSENSORPARAMS Tests n sensor measures, and calculates average and
%covariance of the measures
center_measures=zeros(1,nMeasures);
if landmark
    dist_measures=zeros(1,nMeasures);
end
for i = 1:nMeasures
    if landmark ~= true
        meas=apoloGetLaserData(sensorName);
        siz=size(meas);
        if siz(2) > 181 %LMS100
            %center_measures(i) = meas(round(539/2));
        else %LMS200
            meas = meas(1:180); %measure 181 is always 0...
        end
        center_measures(i) = meas(round(size(meas,2)/2)); %get the forward facing laser
    else
        meas=apoloGetLaserLandMarks(sensorName);
        center_measures(i)=meas.angle(1);
        dist_measures(i)=meas.distance(1);
    end
    %pause(0.5);
    apoloUpdate();
end
avg = mean(center_measures);
%Plot results
figure("Name","Calibration results");
subplot(1,2,1),title("Measures through time"),plot(center_measures);
subplot(1,2,2),title("Measure distribution"),histogram(center_measures);
xline(avg,'--r',{'Average','= '+string(avg)});

cov = std(center_measures);
if landmark
    avgD=mean(dist_measures);
    covD=std(dist_measures);
end