


%initialise
done = 0;
state = 'reset';
cube_num = 1;
% cube positions, we need to pass these off as goalcoord to integrate with
% existing code
coord_cube1 = [1,1,1];
coord_cube2 = [1,1,1];
coord_cube3 = [1,1,1];
coord_rotate = [1,1,1];
coord_reset = [27.4,20.5,0];

%end coordinates
coord_cube1_end = [1,2,3];
coord_cube2_end = [1,2,3];
coord_cube3_end = [1,2,3];
%how many rotations do we need for each cube?
rotate_cube1 = 1;
rotate_cube2 = 2;
rotate_cube3 = 2;
rotation_num = 0;
%pick up poses
pose_pickup = -90;
pose_rotate = 0;
pose_reset = 0;
gripper_open = 0;
gripper_close = 1;
%rotate coords
coord_rotate = [1,1,1];
goalcoord = 0;
goalpose = 0;
gripper_pos = 0;



while done == 0
    
    switch state
        
        case 'reset'
            %do something to be reset/default state
            disp('finished reset state')
            cube_num
            move(coord_reset, pose_reset,gripper_open);
            state = 'pick n grab'
            
        case 'pick n grab'
            
            if cube_num == 1 
                goalcoord = coord_cube1;
            end
            if cube_num == 2 
                goalcoord = coord_cube1;
            end
            if cube_num == 3 
                goalcoord = coord_cube1;
            end
            
            goalpose = pose_pickup;
            gripper_pos = gripper_open;
            %move and pick
            move(goalcoord, goalpose,gripper_pos);
            gripper_pos = gripper_close;
            move(goalcoord, goalpose,gripper_pos);

            disp('pick n grab')
            cube_num

            
            state = 'rotate'
            
        case 'rotate'
            
            if cube_num == 1
                rotation_num = rotate_cube1;
            end
            if cube_num == 2
                rotation_num = rotate_cube2;
            end
            if cube_num == 3
                rotation_num = rotate_cube3;
            end
            
            %go to some place to rotate the cube to not collide with
            %anything, 
            
            
            for i = 1:rotation_num

               %move to rotate coord+rotate
               goalcoord = coord_rotate;
               goalpose = pose_rotate;
               gripper_pos = gripper_close;
               move(goalcoord, goalpose,gripper_pos);

               gripper_pos = gripper_open; 
                
               move(goalcoord, goalpose,gripper_pos);

               goalcoord = coord_rotate;
               goalpose = pose_pickup;
               move(goalcoord, goalpose,gripper_pos);

               gripper_pos = gripper_close;
               move(goalcoord, goalpose,gripper_pos);

            end

            disp('rotate')
            cube_num
            rotation_num
     
            state = 'place end'
            
        case 'place end'
            
            if cube_num == 1
                goalcoord = coord_cube1_end
            end
            if cube_num == 2
                goalcoord = rotate_cube2_end;
            end
            if cube_num == 3
                goalcoord = rotate_cube3_end;
            end
            
            goalpose = pose_pickup;
            gripper_pos = gripper_close;
            move(goalcoord, goalpose,gripper_pos);
            gripper_pos = gripper_open;
            move(goalcoord, goalpose,gripper_pos);
            
            if cube_num == 3
                done = 1;
            end
            
            cube_num = cube_num + 1;  

            disp('finished reset state')
            cube_num
            
    end  



    function move(intervalcoord, intervalpose,gripper_pos)

        
        base2goal=atan2d(intervacoord(3),intervalcoord(1));
        xzdist=sqrt(intervalcoord(3)^2+intervalcoord(1)^2);
        %p1,p2,p3
        p3=[xzdist,intervalcoord(2)];
        p2=p3-[L3*cosd(intervalpose),L3*sind(intervalpose)];
    
        p2=p2-[0,7.7];
    
        goalxz=p2(1);
        goaly=p2(2);
    
    
        c2=(goalxz^2+goaly^2-L1^2-L2^2)/(2*L1*L2);
        s2up=sqrt(1-c2^2);
        s2down=-sqrt(1-c2^2);
    
        k1=L1+L2*c2;
        k2up=L2*s2up;
        k2down=L2*s2down;
    
        % theta2up=atan2(s2up,c2);
        theta2down=atan2(s2down,c2);
        % theta1up=-atan2(goalx,goaly)-atan2(k2up,k1)+pi/2;%-atan2(128,24);
        theta1down=-atan2(goalxz,goaly)-atan2(k2down,k1)+pi/2;
    
        base=[0,0,0];
    
        node1=[0,7.7,0];
        node2=node1+[L1*cosd(base2goal)*cos(theta1down),L1*sin(theta1down),L1*sind(base2goal)*cos(theta1down)];
        node3=node2+[L2*cosd(base2goal)*cos(theta1down+theta2down),L2*sin(theta1down+theta2down),L2*sind(base2goal)*cos(theta1down+theta2down)];
        node4=node3+[L3*cosd(base2goal)*cos(intervalpose*pi/180),L3*sin(intervalpose*pi/180),L3*sind(base2goal)*cos(intervalpose*pi/180)];
    
    
        theta1_IK=base2goal*pi/180;
        theta2_IK=theta1down-atan2(12.8,2.4);
        theta3_IK=theta1down+theta2down-theta2_IK;
        theta4_IK= intervalpose*pi/180- theta2down-theta1down;
    
        theta1_IK = (theta1_IK *180/pi);
        theta2_IK = (theta2_IK *180/pi);
        theta3_IK = (theta3_IK *180/pi);
        theta4_IK = (theta4_IK *180/pi);
    
        encoder_theta0 = -(theta1_IK*2048 )/ 180 +2048;
        encoder_theta1 = -(theta2_IK*2048 )/ 180 +2048 ;
        encoder_theta2 = -(theta3_IK*2048 )/ 180 +2048;
        encoder_theta3 = -(theta4_IK*2048 )/ 180 +2048;
    
        if(gripper_pos == 1)
            encoder_theta4 = 216*2048/180;
        end
        if(gripper_pos == 0)
            encoder_theta4 = 1024;
    end



    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_GOAL_POSITION, encoder_theta0);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_GOAL_POSITION, encoder_theta1);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PRO_GOAL_POSITION, encoder_theta2);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PRO_GOAL_POSITION, encoder_theta3);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_PRO_GOAL_POSITION, encoder_theta4);



    end