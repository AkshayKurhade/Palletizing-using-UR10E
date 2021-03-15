function moveL(clientID, target, pos, speed)
    vrep=remApi('remoteApi');

    %Get target postion and orientation
    [r,p]=vrep.simxGetObjectPosition(clientID,target,-1,vrep.simx_opmode_blocking);
    [r,o]=vrep.simxGetObjectOrientation(clientID,target,-1,vrep.simx_opmode_blocking);

    %either +ve or -ve direction available, calculating shortest one
    for i=1:3
        if ((abs(pos(i+3)-o(i))> pi) && (o(i)<0))
            o(i)=o(i)+2*pi;
        elseif ((abs(pos(i+3)-o(i))> pi) && (o(i)>0))
            o(i)=o(i)+2*pi;
        end
    end
    old_pos=[p o];
    delta_pos=pos-old_pos;
    distance=norm(delta_pos);
    samples_number=round(distance*50);
    for i=1:samples_number
        intermediate_pos=old_pos+ (delta_pos/samples_number);
        tic;
        while(toc< (distance/(speed*samples_number)))
        end
        
        %Set intermediate position of the object
        vrep.simxSetObjectPosition(clientID,target,-1,intermediate_pos,vrep.simx_opmode_blocking);
        vrep.simxSetObjectOrientation(clientID,target,-1,intermediate_pos(4:6),vrep.simx_opmode_blocking);
        old_pos=intermediate_pos;
    end    
end
