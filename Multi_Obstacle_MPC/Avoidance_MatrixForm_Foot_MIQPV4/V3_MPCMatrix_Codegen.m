clear 
clc 

addpath('/home/orl/Matlab_Work/casadi-3.6.6-linux64-matlab2018b')
import casadi.*

for n = 0:3
    step_index = n;
    Casadi_Avoidance_Matrix_LeftStartV3(step_index)
    Casadi_Avoidance_Matrix_RightStartV3(step_index)
end

%%
mex LeftStart_Step0V3.c -largeArrayDims
mex LeftStart_Step1V3.c -largeArrayDims
mex LeftStart_Step2V3.c -largeArrayDims
mex LeftStart_Step3V3.c -largeArrayDims

mex RightStart_Step0V3.c -largeArrayDims
mex RightStart_Step1V3.c -largeArrayDims
mex RightStart_Step2V3.c -largeArrayDims
mex RightStart_Step3V3.c -largeArrayDims

%%
movefile('LeftStart_Step0V3.c','Generated_Function/LeftStart_Step0V3.c')
movefile('LeftStart_Step1V3.c','Generated_Function/LeftStart_Step1V3.c')
movefile('LeftStart_Step2V3.c','Generated_Function/LeftStart_Step2V3.c')
movefile('LeftStart_Step3V3.c','Generated_Function/LeftStart_Step3V3.c')
movefile('RightStart_Step0V3.c','Generated_Function/RightStart_Step0V3.c')
movefile('RightStart_Step1V3.c','Generated_Function/RightStart_Step1V3.c')
movefile('RightStart_Step2V3.c','Generated_Function/RightStart_Step2V3.c')
movefile('RightStart_Step3V3.c','Generated_Function/RightStart_Step3V3.c')


%%
disp('Generating mpc casadi lib')
cd Generated_Function/
system('./create_lib.sh');
cd ..