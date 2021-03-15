clear all;
close all;
clc

origine_table=[0.95,-0.725,0.3632]; %%Origin of the pallet
cube_dimensions=[0.1,0.1,0.1];      %%dimensions of the work objeect
width_offset=0.05;                  %%width offsets between objects
length_offset=0.05;                 %%length offsets between objects
end_test=0;
j_variable=0;

% Initialising Vrep remoteAPI
vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);


%Main Code
if(clientID>-1)
    disp('connected to remote API server');
    %object handles
    for i=0:100
        j_variable=i;
     %Get object Handles from the api for gripper,target, proximity sensor 
    [res,j1]=vrep.simxGetObjectHandle(clientID,'ROBOTIC_85_active1',vrep.simx_opmode_blocking);
    [res,j2]=vrep.simxGetObjectHandle(clientID,'ROBOTIC_85_active2',vrep.simx_opmode_blocking);
    [res,ur_target]=vrep.simxGetObjectHandle(clientID,'target',vrep.simx_opmode_blocking);
    [res,Proximity_sensor]=vrep.simxGetObjectHandle(clientID,'Proximity_sensor',vrep.simx_opmode_blocking);
    
    
    %manually defining the initial travel path
    fposition1=[-0.143,-0.188,0.917,0,0,0];
    fposition2=[0.2,0.6,0.6,0,0,0];
    fposition3=[0.000078,-0.664,1.0,0,0,0];
    fposition4=[0.000078,-0.664,0.510,0,0,0];
    fposition5=[0.575,-0.725,1.0,0,0,0];
    fposition6=[0.575,-0.725,0.8,0,0,0];
    
    gripper(clientID,0,j1,j2); pause(1.5);
    moveL(clientID, ur_target, fposition3,2);%open gripper
    while(end_test==0)
        [res,Psensor_distance,detectedPoint]=vrep.simxReadProximitySensor(clientID,Proximity_sensor,vrep.simx_opmode_blocking);
        if (Psensor_distance>0)
             moveL(clientID, ur_target, fposition4,2);  %%move to pickup position
             gripper(clientID,1,j1,j2);pause(2);        %close gripper/ pick object
             moveL(clientID, ur_target, fposition3,2);  %move to position above object
             moveL(clientID, ur_target, fposition5,2);  %move to position above drop location
             moveL(clientID, ur_target, fposition6,2);  %move to drop location
             gripper(clientID,0,j1,j2);pause(1);         %open the gripper and drop object
             moveL(clientID, ur_target, fposition3,2);  %move to initial position
             [end_test,fposition6,fposition5,fposition3]=pick_and_place(origine_table,4,4,3,cube_dimensions,width_offset,length_offset,fposition6,fposition5,fposition3);
        end
    end
    vrep.delete();
    disp('program terminated'); %%terminate the program once pallet is full
    end
end
        
    