clear; % Clear variables
datasetNum = 9; % CHANGE THIS VARIABLE TO CHANGE DATASET_NUM
[sampledData, sampledVicon, sampledTime] = init(datasetNum);
Z = sampledVicon(1:6,:); %all the measurements that you need for the update
% Set initial condition
uPrev = vertcat(sampledVicon(1:9,1),zeros(6,1)); % Copy the Vicon Initial state
covarPrev = eye(15); % Covariance constant
savedStates = zeros(15, length(sampledTime)); % Just for saving state his.
prevTime = 0; %last time step in real time
%write your code here calling the pred_step.m and upd_step.m functions
for i = 1:length(sampledTime)
    % determine dt
    dt=sampledTime(i)-prevTime;
    prevTime=sampledTime(i); %for the next loop

    % Inputs
    angVel=sampledData(i).omg;
    acc=sampledData(i).acc;

    [covar_est,u_est]=pred_step(uPrev,covarPrev,angVel,acc,dt);
    [u_curr,covar_curr]=upd_step(Z(:,i),covar_est,u_est);
    
    uPrev=u_curr;
    covarPrev=covar_curr;
    savedStates(:,i)=u_curr;

end
plotData(savedStates, sampledTime, sampledVicon, 1, datasetNum);