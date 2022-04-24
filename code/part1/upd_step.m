function [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst)
%z_t is the measurement
%covarEst and uEst are the predicted covariance and mean respectively
%uCurr and covar_curr are the updated mean and covariance respectively
Ct = [eye(3), zeros(3,12);zeros(3,3), eye(3), zeros(3,9)];
%Wt not needed as measurement model is linear
persistent Rt
if isempty(Rt) %Everytime you run the code the R matrix changes but stays constant through iterations
    Rt_values=rand(6,1); %Randomize Rt covariance matrix elements so that all the diagonal terms are different
    
    %Make the Rt values bigger or smaller by changing "/100" above
    
    %Rt_values=[1,2,3,2,3,4,3,2,1,2,3,1,2,3,1]%change this value and test (noise variance)
    Rt=diag(Rt_values);
end

Kt=covarEst*transpose(Ct)*inv(Ct*covarEst*transpose(Ct)+Rt);
covar_curr=covarEst-Kt*Ct*covarEst;
uCurr=uEst+Kt*(z_t-Ct*uEst);

end