clc; clear; 
close all;
pkg load statistics
%Choose object detection probability
P_D = 0.9;
%Choose clutter rate
lambda_c = 10;
%Create sensor model
range_c = [-1000 1000;-1000 1000];
sensor_model = modelgen.sensormodel(P_D,lambda_c,range_c);
%Create ground truth model
nbirths = 1;
K = 100;
initial_state.x = [0; 0; 10; 10];
initial_state.P = eye(4);
ground_truth = modelgen.groundtruth(nbirths,initial_state.x,1,K+1,K);
%Create linear motion model
T = 1;
sigma_q = 5;
motion_model = motionmodel.cvmodel(T,sigma_q);
        
%Create linear measurement model
sigma_r = 10;
meas_model = measmodel.cvmeasmodel(sigma_r);
%Generate true object data (noisy or noiseless) and measurement data
ifnoisy = 0;
objectdata = objectdatagen(ground_truth,motion_model,ifnoisy);
measdata = measdatagen(objectdata,sensor_model,meas_model);
saveDataInFiles(measdata);

