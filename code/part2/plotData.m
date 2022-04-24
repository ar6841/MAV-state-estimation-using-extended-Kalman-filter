function plotData(savedStates, sampledTime, sampledVicon, modelNum, datasetName)
%PLOTDATA Plot the predicted data
xPos = savedStates(1,:);
yPos = savedStates(2,:);
zPos = savedStates(3,:);
xOrient = savedStates(4,:);
yOrient = savedStates(5,:);
zOrient = savedStates(6,:);
xVel = savedStates(7,:);
yVel = savedStates(8,:);
zVel = savedStates(9,:);
xBg = savedStates(10,:);
yBg = savedStates(11,:);
zBg = savedStates(12,:);
xBa = savedStates(13,:);
yBa = savedStates(14,:);
zBa = savedStates(15,:);

figure('Name', sprintf('Model %d - Dataset %d', modelNum, datasetName));
%%
subplot(5,3,1);
plot(sampledTime, xPos,'r', sampledTime, sampledVicon(1,:),'b');
title('Position X');
legend('Predicted', 'Actual');
subplot(5,3,4);
plot(sampledTime, xOrient, 'r',sampledTime, sampledVicon(4,:),'b');
title('Orientation X');
subplot(5,3,7);
plot(sampledTime, xVel, 'r',sampledTime, sampledVicon(7,:),'b');
title('Velocity X');
subplot(5,3,10);
plot(sampledTime, xBg, 'r');
title('Bias Gyroscope X');
subplot(5,3,13);
plot(sampledTime, xBa, 'r');
title('Bias Accelerometer X');
xlabel('Time (s)');
%%
subplot(5,3,2);
plot(sampledTime, yPos,'r', sampledTime, sampledVicon(2,:),'b');
title('Position Y');
subplot(5,3,5);
plot(sampledTime, yOrient, 'r',sampledTime, sampledVicon(5,:),'b');
title('Orientation Y');
subplot(5,3,8);
plot(sampledTime, yVel, 'r',sampledTime, sampledVicon(8,:),'b');
title('Velocity Y');
subplot(5,3,11);
plot(sampledTime, yBg, 'r');
title('Bias Gyroscope Y');
subplot(5,3,14);
plot(sampledTime, yBa, 'r');
title('Bias Accelerometer Y');
xlabel('Time (s)');

%%
subplot(5,3,3);
plot(sampledTime, zPos,'r', sampledTime, sampledVicon(3,:),'b');
title('Position Z');
subplot(5,3,6);
plot(sampledTime, zOrient, 'r',sampledTime, sampledVicon(6,:),'b');
title('Orientation Z');
subplot(5,3,9);
plot(sampledTime, zVel, 'r',sampledTime, sampledVicon(9,:),'b');
title('Velocity Z');
subplot(5,3,12);
plot(sampledTime, zBg, 'r');
title('Bias Gyroscope Z');
subplot(5,3,15);
plot(sampledTime, zBa, 'r');
title('Bias Accelerometer Z');
xlabel('Time (s)');
end


