clc;
clear all;

lib_name = '';

if strcmp(computer, 'PCWIN')
  lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
  lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
  lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
  lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end

%% ---- Control Table Addresses ---- %%

ADDR_PRO_TORQUE_ENABLE       = 64;           % Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION       = 116; 
ADDR_PRO_PRESENT_POSITION    = 132; 
ADDR_PRO_OPERATING_MODE      = 11;

%% ---- Other Settings ---- %%

% Protocol version
global PROTOCOL_VERSION;
global ADDR_PRO_PRESENT_POSITION ADDR_PRO_GOAL_POSITION;
global DXL_ID_1 DXL_ID_2 DXL_ID_3 DXL_ID_4 DXL_ID_5;
PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel

% Default setting
DXL_ID_1                      =11;           
DXL_ID_2                      =12;
DXL_ID_3                      =13;           
DXL_ID_4                      =14;
DXL_ID_5                      =15;           

BAUDRATE                    = 115200;
DEVICENAME                  = 'COM11';       % Check which port is being used on your controller
                                            % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'
                                            
TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = -150000;      % Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 150000;       % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold

ESC_CHARACTER               = 'e';          % Key for escaping loop

COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed

%% ------------------ %%

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
global port_num;
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

index = 1;
dxl_comm_result = COMM_TX_FAIL;           % Communication result
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE DXL_MAXIMUM_POSITION_VALUE];         % Goal position

dxl_error = 0;                              % Dynamixel error
dxl_present_position = 0;                   % Present position


% Open port
if (openPort(port_num))
    fprintf('Port Open\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port\n');
    input('Press any key to terminate...\n');
    return;
end


% -----SET MOTION LIMITS -----------%
ADDR_MAX_POS = 48;
ADDR_MIN_POS = 52;
MAX_POS = 4048;
MIN_POS = 0;
% Set max position limit
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_MAX_POS, MAX_POS);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_MAX_POS, MAX_POS);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_MAX_POS, MAX_POS);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_MAX_POS, MAX_POS);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_MAX_POS, MAX_POS);

% Set min position limit
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_MIN_POS, MIN_POS);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_MIN_POS, MIN_POS);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_MIN_POS, MIN_POS);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_MIN_POS, MIN_POS);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_MIN_POS, MIN_POS);
%velocity
% address 10 to value 4

write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, 10, 4);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, 10, 4);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, 10, 4);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, 10, 4);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, 10, 4);


% ----------------------------------%



% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Baudrate Set\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end

% Put actuator into Position Control Mode
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_OPERATING_MODE, 4);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_OPERATING_MODE, 4);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PRO_OPERATING_MODE, 4);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PRO_OPERATING_MODE, 4);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_PRO_OPERATING_MODE, 4);

% Disable Dynamixel Torque
%write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_TORQUE_ENABLE, 1);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_TORQUE_ENABLE, 1);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PRO_TORQUE_ENABLE, 1);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PRO_TORQUE_ENABLE, 1);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_PRO_TORQUE_ENABLE, 1);

write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, 112, 1000);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, 112, 1000);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, 112, 1000);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, 112, 1000);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, 112, 1000);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, 108, 1000);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, 108, 1000);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, 108, 1000);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, 108, 1000);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, 108, 1000);

%initialise
done = 0;
state = 'reset';
cube_num = 1;
% cube positions, we need to pass these off as goalcoord to integrate with
% existing code
coord_cube1 = [22.5,6,0];
coord_cube2 = [7,6,20];
coord_cube3 = [15.3,6,-15];
coord_reset = [27.4,20.5,0];
%elevations
coord_cube1_elevate = coord_cube1 + [0,0.8,0];
coord_cube2_elevate = coord_cube2 + [0,2,0];
coord_cube3_elevate = coord_cube3 + [0,2,0];

%end coordinates
coord_cube1_end = [10,5.5,0];
coord_cube2_end = [12.5,5.5,12.5];
coord_cube3_end = [-0.2,5.5,-10];
%elevations
coord_cube1_end_elevate = coord_cube1_end + [0,2,0];
coord_cube2_end_elevate = coord_cube2_end + [0,2,0];
coord_cube3_end_elevate = coord_cube3_end + [0,2,0];
%how many rotations do we need for each cube?
rotate_cube1 = 0;
rotate_cube2 = 0;
rotate_cube3 = 0;
rotation_num = 1;
%pick up poses
pose_pickup = -90;
pose_rotate = 0;
pose_reset = 0;
gripper_open = 0;
gripper_close = 1;
%rotate coords
coord_rotate = [22.5,5.5,0];
goalcoord = 0;
goalpose = 0;
gripper_pos = 0;
goalcoord_elevate=0;


while done == 0
    
    switch state
        
        case 'reset'
            %do something to be reset/default state
            disp('finished reset state')
            cube_num
            move(coord_reset, pose_reset,gripper_open);
            state = 'pick n grab'
            
        case 'pick n grab'
            disp('second state')
            if cube_num == 1 
                goalcoord = coord_cube1;
                goalcoord_elevate = coord_cube1_elevate;
            end
            if cube_num == 2 
                goalcoord = coord_cube2;
                goalcoord_elevate = coord_cube2_elevate;
            end
            if cube_num == 3 
                goalcoord = coord_cube3;
                goalcoord_elevate = coord_cube3_elevate;
            end
            

            goalpose=pose_pickup;
            gripper_pos = gripper_open;

            %move to elevated position
            move(goalcoord_elevate,goalpose,gripper_pos)

            %move and pick
            move(goalcoord, goalpose,gripper_pos);
            gripper_pos = gripper_close;
            move(goalcoord, goalpose,gripper_pos);

            %move to elevated position
            move(goalcoord_elevate,goalpose,gripper_pos)

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
                goalcoord_elevate = coord_cube1_end_elevate;

            end
            if cube_num == 2
                goalcoord = coord_cube2_end;
                goalcoord_elevate = coord_cube2_end_elevate;

            end
            if cube_num == 3
                goalcoord = coord_cube3_end;
                goalcoord_elevate = coord_cube3_end_elevate;

            end
            
            goalpose = pose_pickup;
            gripper_pos = gripper_close;
            move(goalcoord_elevate, goalpose,gripper_pos);
            move(goalcoord, goalpose,gripper_pos);
            gripper_pos = gripper_open;
            move(goalcoord, goalpose,gripper_pos);
            move(goalcoord_elevate, goalpose,gripper_pos);

            
            if cube_num == 3
                done = 1;
            end
            
            cube_num = cube_num + 1;  

            disp('finished reset state')
            cube_num

            state = 'reset'

        otherwise 
            disp('error')
            
    end  

end

function move(intervalcoord, intervalpose,gripper_pos)
global port_num;
global PROTOCOL_VERSION;
global ADDR_PRO_PRESENT_POSITION ADDR_PRO_GOAL_POSITION;
global DXL_ID_1 DXL_ID_2 DXL_ID_3 DXL_ID_4 DXL_ID_5;

count = 100;
maxerror=1.21;
minerror=0.9;


L3=12.6;
L2=12.4;
L1=13;


base2goal=atan2d(intervalcoord(3),intervalcoord(1));
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



% a = 0;
% b=0;
% c=0;
% d=0;
% 
% theta1=(-180/2048)*(read4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID_1,ADDR_PRO_PRESENT_POSITION)-2048);
% theta2=(-180/2048)*(read4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID_2,ADDR_PRO_PRESENT_POSITION)-2048);
% theta3=(-180/2048)*(read4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID_3,ADDR_PRO_PRESENT_POSITION)-2048);
% theta4=(-180/2048)*(read4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID_4,ADDR_PRO_PRESENT_POSITION)-2048);
% 
% prevangle=[theta1,theta2,theta3,theta4];
% 
% goal_theta1 = theta1_IK;
% goal_theta2 = theta2_IK;
% goal_theta3 = theta3_IK;
% goal_theta4 = theta4_IK;
% error = 2;
% 
% 
% count1 = 10;
% count2 = 10;
% count3 = 10;
% count4 = 10;
% 
% 
% while (a==0 || b==0 || c==0 || d==0)
%     
% 
%     present_theta1=(-180/2048)*(read4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID_1,ADDR_PRO_PRESENT_POSITION)-2048);
%     present_theta2=(-180/2048)*(read4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID_2,ADDR_PRO_PRESENT_POSITION)-2048);
%     present_theta3=(-180/2048)*(read4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID_3,ADDR_PRO_PRESENT_POSITION)-2048);
%     present_theta4=(-180/2048)*(read4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID_4,ADDR_PRO_PRESENT_POSITION)-2048);
% 
%     delta1=goal_theta1-prevangle(1);
%     delta2=goal_theta2-prevangle(2);
%     delta3=goal_theta3-prevangle(3);
%     delta4=goal_theta4-prevangle(4);
% 
%     % disp([present_theta1,theta1_IK])
%     % disp([present_theta2,theta2_IK])
%     % disp([present_theta3,theta3_IK])
%     % disp([present_theta4,theta4_IK])
%     % disp('%%%%%%%%%%%%%%%%')
% 
% 
%     if(theta1_IK >= present_theta1)
%         %     if (present_theta1 > theta1_IK*maxerror || present_theta1 < theta1_IK*minerror)
%         %         a = 0;
%         %         theta1 = theta1 + abs(delta1)/count;
%         %     end
%         if (abs(present_theta1) <= (abs(theta1_IK)+error) && abs(present_theta1) >= (abs(theta1_IK)-error) && (sign(present_theta1) == sign(theta1_IK+2) || sign(present_theta1) == sign(theta1_IK-2)))
%             a = 1;
%             disp('a1')
%         else
%             if(abs(theta1_IK-present_theta1)<(abs(delta1)/count1))
%                 count1 = count1*3;
%             end
%             a = 0;
%             theta1 = theta1 + abs(delta1)/count1;
%         end
%     else
%         % if(theta1_IK < present_theta1)
%         %     if (present_theta1 > theta1_IK*maxerror || present_theta1 < theta1_IK*minerror)
%         %         a = 0;
%         %         theta1 = theta1 - abs(delta1)/count;
%         %     end
%         if (abs(present_theta1) <= (abs(theta1_IK)+error) && abs(present_theta1) >= (abs(theta1_IK)-error) && (sign(present_theta1) == sign(theta1_IK+2) || sign(present_theta1) == sign(theta1_IK-2)))
%             a = 1;
%             disp('a2')
%         else
%             if(abs(theta1_IK-present_theta1)<(abs(delta1)/count1))
%                 count1 = count1*3;
%             end
%             a = 0;
%             theta1 = theta1 - abs(delta1)/count1;
%         end
%     end
% 
%     if(theta2_IK >= present_theta2)
%         if (abs(present_theta2) <= (abs(theta2_IK)+error) && abs(present_theta3) >= (abs(theta2_IK)-error) && (sign(present_theta2) == sign(theta2_IK+2) || sign(present_theta2) == sign(theta2_IK-2)))
%             b = 1;
%             disp('b1')
%         else
%             if(abs(theta2_IK-present_theta2)<(abs(delta2)/count2))
%                 count2 = count2*3;
%             end
%             b = 0;
%             theta2 = theta2 + abs(delta2)/count2;
%         end
%     else
%         if (abs(present_theta2) <= (abs(theta2_IK)+error) && abs(present_theta2) >= (abs(theta2_IK)-error) && (sign(present_theta2) == sign(theta2_IK+2) || sign(present_theta2) == sign(theta2_IK-2)))
%             b = 1;
%             disp('b2')
%         else
%             if(abs(theta2_IK-present_theta2)<(abs(delta2)/count2))
%                 count2 = count2*3;
%             end
%             b = 0;
%             theta2 = theta2 - abs(delta2)/count2;
%         end
%     end
% 
%     if(theta3_IK >= present_theta3)
%         if (abs(present_theta3) <= (abs(theta3_IK)+error) && abs(present_theta3) >= (abs(theta3_IK)-error) && (sign(present_theta3) == sign(theta3_IK+2) || sign(present_theta3) == sign(theta3_IK-2)))
%             c = 1;
%             disp('c1')
%         else
%             if(abs(theta3_IK-present_theta3)<(abs(delta3)/count3))
%                 count3 = count3*3;
%             end
%             c = 0;
%             theta3 = theta3 + abs(delta3)/count3;
%         end
%     else
%         if (abs(present_theta3) <= (abs(theta3_IK)+error) && abs(present_theta3) >= (abs(theta3_IK)-error) && (sign(present_theta3) == sign(theta3_IK+2) || sign(present_theta3) == sign(theta3_IK-2)))
%             c = 1;
%             disp('c2')
%         else
%             if(abs(theta3_IK-present_theta3)<(abs(delta3)/count3))
%                 count3 = count3*3;
%             end
%             c = 0;
%             theta3 = theta3 - abs(delta3)/count3;
%         end
%     end
% 
%     if(theta4_IK >= present_theta4)
%         if (abs(present_theta4) <= (abs(theta4_IK)+error) && abs(present_theta4) >= (abs(theta4_IK)-error) && (sign(present_theta4) == sign(theta4_IK+2) || sign(present_theta4) == sign(theta4_IK-2)))
%             d = 1;
%             disp('d1')
%         else
%             if(abs(theta4_IK-present_theta4)<(abs(delta4)/count4))
%                 count4 = count4*3;
%             end
%             d = 0;
%             theta4 = theta4 + abs(delta4)/count4
%         end
%     else
%         if (abs(present_theta4) <= (abs(theta4_IK)+error) && abs(present_theta4) >= (abs(theta4_IK)-error) && (sign(present_theta4) == sign(theta4_IK+2) || sign(present_theta4) == sign(theta4_IK-2)))
%             d = 1;
%             disp('d2')
%         else
%             if(abs(theta4_IK-present_theta4)<(abs(delta4)/count4))
%                 count4 = count4*3;
%             end
%             d = 0;
%             theta4 = theta4 - abs(delta4)/count4
%         end
%     end

encoder_theta0 = -(theta1_IK*2048 )/ 180 +2048
encoder_theta1 = -(theta2_IK*2048 )/ 180 +2048
encoder_theta2 = -(theta3_IK*2048 )/ 180 +2048
encoder_theta3 = -(theta4_IK*2048 )/ 180 +2048

write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_GOAL_POSITION, encoder_theta0);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_GOAL_POSITION, encoder_theta1);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PRO_GOAL_POSITION, encoder_theta2);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PRO_GOAL_POSITION, encoder_theta3);
pause(0.01)
% end

if(gripper_pos == 1)
    encoder_theta4 = 216*2048/180;
end
if(gripper_pos == 0)
    encoder_theta4 = 1024;
end
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_PRO_GOAL_POSITION, encoder_theta4);
pause(0.3)
end