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
DEVICENAME                  = 'COM5';       % Check which port is being used on your controller
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


% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Baudrate Set\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end

write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_TORQUE_ENABLE, 0);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_TORQUE_ENABLE, 0);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PRO_TORQUE_ENABLE, 0);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PRO_TORQUE_ENABLE, 0);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_PRO_TORQUE_ENABLE, 0);

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



speed = 700;
acceleration = 50;
pause(0.5)

write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, 10, 4);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, 10, 4);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, 10, 4);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, 10, 4);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, 10, 4);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, 11, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, 11, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, 11, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, 11, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, 11, 3);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, 112, speed);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, 112, speed);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, 112, speed);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, 112, speed);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, 112, speed);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, 108, acceleration);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, 108, acceleration);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, 108, acceleration);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, 108, acceleration);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, 108, acceleration);


% ----------------------------------%




% Put actuator into Position Control Mode
% write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_OPERATING_MODE, 4);
% write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_OPERATING_MODE, 4);
% write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PRO_OPERATING_MODE, 4);
% write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PRO_OPERATING_MODE, 4);
% write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_PRO_OPERATING_MODE, 4);

% Disable Dynamixel Torque
%write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_TORQUE_ENABLE, 1);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_TORQUE_ENABLE, 1);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PRO_TORQUE_ENABLE, 1);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PRO_TORQUE_ENABLE, 1);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_PRO_TORQUE_ENABLE, 1);




% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, 10, 4);
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, 10, 4);
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, 10, 4);
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, 10, 4);
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, 10, 4);
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, 11, 3);
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, 11, 3);
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, 11, 3);
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, 11, 3);
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, 11, 3);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, 112, speed);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, 112, speed);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, 112, speed);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, 112, speed);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, 112, speed);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, 108, acceleration);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, 108, acceleration);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, 108, acceleration);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, 108, acceleration);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, 108, acceleration);

%initialise
done = 0;
state = 'reset';
cube_num = 1;
% cube positions, we need to pass these off as goalcoord to integrate with
% existing code
coord_cube1 = [22.5,3.75,0];
coord_cube2 = [7.5,3.75,20];
coord_cube3 = [15,3.75,-15];
coord_reset = [27.4,20.5,0];
%elevations
coord_cube1_elevate = coord_cube1 + [0,0.8,0];
coord_cube2_elevate = coord_cube2 + [0,2,0];
coord_cube3_elevate = coord_cube3 + [0,2,0];

%end coordinates
coord_cube1_end = [10,3.75,0];
coord_cube2_end = [12.5,3.75,12.5];
coord_cube3_end = [-0.25,3.75,-10];
%elevations
coord_cube1_end_elevate = coord_cube1_end + [0,2,0];
coord_cube2_end_elevate = coord_cube2_end + [0,2,0];
coord_cube3_end_elevate = coord_cube3_end + [0,2,0];
%how many rotations do we need for each cube?
rotate_cube1 = 1;
rotate_cube2 = 2;
rotate_cube3 = 1;
rotation_num = 1;
%pick up poses
pose_pickup = -35;
pose_rotate = 0;
pose_reset = 0;
gripper_open = 0;
gripper_close = 1;
%rotate coords
coord_rotate = [15,4,-15];
coord_rotate_elevate = coord_rotate + [0,2,0];
coord_rotate_elevate2=coord_rotate_elevate + [-5,4,5]
coord_extend = coord_rotate_elevate2 + [5,0,-5]
goalcoord = 0;
goalpose = 0;
gripper_pos = 0;
goalcoord_elevate=0;




%define lines
coord_pen = [0,4,21];
coord_pen_preelevate = coord_pen + [0,3,-7];
coord_pen_elevate=coord_pen +[0,8,0];
line_steps = 5;
arc_steps = 25;
draw_height = 2.25;
line1x = linspace(1,1,line_steps);
line1y = linspace(1,1,line_steps);
line2x = linspace(1,1,line_steps);
line2y = linspace(1,1,line_steps);
line3x = linspace(1,1,line_steps);
line3y = linspace(1,1,line_steps);

% startpoint=[20,5,6];

startpointx=0;%startpoint(1);
startpointy=0;%startpoint(2);
verticesx=startpointx+[20,20,12.5,20,20];
verticesz=startpointy+[6,14,14,6,14];
operation=[1,1,1,2];%1 for line, 2 for arc

drawcount=1;

%define circle
global startpos endpos r centreabove clockwise
r=4;
centreabove=1;% +1 for centre above and -1 for centre below
clockwise=1;

while done == 0
    
    switch state


        case 'reset'
            %do something to be reset/default state
            disp('finished reset state')
            move(coord_reset, pose_reset,gripper_open);
            state = 'grab pen';
            

        case 'grab pen'
            
            goalcoord = coord_pen;
            goalcoord_preelevate = coord_pen_preelevate;
            goalcoord_elevate = coord_pen_elevate;
            goalpose=pose_pickup;
            gripper_pos = gripper_open;

            %move to elevated position
            move(goalcoord_preelevate,goalpose,gripper_open)
            pause(1)
              
            %move and pick
            move(goalcoord, goalpose,gripper_open);
            pause(1)
            gripper_pos = gripper_close;
            move(goalcoord, goalpose,gripper_close);
            pause(1)

            %move to elevated position
            move(goalcoord_elevate,goalpose,gripper_close)
            pause(1)
            move(coord_reset,0,gripper_close)
            move([verticesx(1),3.75,-verticesz(1)],goalpose,gripper_close)
            pause(0.5);
            state = 'draw'

        case 'draw'
            if(drawcount>4)
                done=1;
                break;
            end
            if(operation(drawcount)==1)
                state = 'line'
            end
            if(operation(drawcount)==2)
                state = 'arc'
            end
        
        case 'line'
            linex=linspace(verticesx(drawcount),verticesx(drawcount+1),line_steps);
            linez=linspace(verticesz(drawcount),verticesz(drawcount+1),line_steps);
            for i = 1:line_steps
                
                goalcoord = [linex(i), draw_height, -linez(i)];
                goalpose = pose_pickup;
                gripper_pos = gripper_close;
                move(goalcoord,goalpose,gripper_pos);
                
            end
            drawcount=drawcount+1;
            state = 'draw'
            
        case 'arc'
            global startpos endpos r centreabove clockwise
            startpos=[verticesx(drawcount),verticesz(drawcount)];
            endpos=[verticesx(drawcount+1),verticesz(drawcount+1)];
            d=sqrt((endpos(1)-startpos(1))^2+(endpos(2)-startpos(2))^2);
            dhat=(endpos-startpos)/d;
            phat=centreabove*[dhat(2),-dhat(1)];
            
            % r^2=(d^2/4)+p^2
            p=sqrt(r^2-d^2/4);
            centre=startpos+((d/2)*dhat+p*phat);
            
            c2s=startpos-centre;
            c2sangle=wrapTo360(atan2d(c2s(2),c2s(1)));
            c2e=endpos-centre;
            c2eangle=wrapTo360(atan2d(c2e(2),c2e(1)));

            
            
            if(clockwise==1)
                if(c2eangle>c2sangle)
                    if(centreabove==-1)
                        c2sangle=c2sangle+360;
                    end
                end
            else
                if(c2eangle<c2sangle)
                    if(centreabove==1)
                        c2eangle=c2eangle+360;
                    end
                end
            end
            % anglelinspace=linspace(c2sangle,c2eangle,200);
            anglelinspace=(linspace(c2sangle,c2eangle,arc_steps));
            
            arcx=r*cosd(anglelinspace)+centre(1);
            arcz=r*sind(anglelinspace)+centre(2);
            goalpose = pose_pickup;
            gripper_pos = gripper_close;
            for i=1:length(arcx)
                move([arcx(i),draw_height,-arcz(i)],goalpose,gripper_pos)
            end
            drawcount=drawcount+1;
            state = 'draw'
           
        otherwise 
            disp('error')
            
    end  

end

function move(intervalcoord, intervalpose,gripper_pos)
global port_num;
global PROTOCOL_VERSION;
global ADDR_PRO_PRESENT_POSITION ADDR_PRO_GOAL_POSITION;
global DXL_ID_1 DXL_ID_2 DXL_ID_3 DXL_ID_4 DXL_ID_5;

count = 10;
maxerror=1.21;
minerror=0.9;


L3=14.66;
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


% 
% a = 0;
% b=0;
% c=0;
% d=0;
% 
% 
% prevangle=[theta1,theta2,theta3,theta4];
% 
% goal_theta1 = theta1_IK;
% goal_theta2 = theta2_IK;
% goal_theta3 = theta3_IK;
% goal_theta4 = theta4_IK;
% error = 0.5;
% 
% 
% count1 = 5;
% count2 = 5;
% count3 = 5;
% count4 = 5;
% 
% 
% while (a==0 || b==0 || c==0 || d==0)
% 
% 
%     %     present_theta1=(-180/2048)*(read4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID_1,ADDR_PRO_PRESENT_POSITION)-2048);
%     %     present_theta2=(-180/2048)*(read4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID_2,ADDR_PRO_PRESENT_POSITION)-2048);
%     %     present_theta3=(-180/2048)*(read4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID_3,ADDR_PRO_PRESENT_POSITION)-2048);
%     %     present_theta4=(-180/2048)*(read4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID_4,ADDR_PRO_PRESENT_POSITION)-2048);
% 
%     delta1=goal_theta1-prevangle(1);
%     delta2=goal_theta2-prevangle(2);
%     delta3=goal_theta3-prevangle(3);
%     delta4=goal_theta4-prevangle(4);
% 
%     present_theta1=goal_theta1;
%     present_theta2=goal_theta2;
%     present_theta3=goal_theta3;
%     present_theta4=goal_theta4;
% 
%     % disp([present_theta1,theta1_IK])
%     % disp([present_theta2,theta2_IK])
%     % disp([present_theta3,theta3_IK])
%     % disp([present_theta4,theta4_IK])
%     % disp('%%%%%%%%%%%%%%%%')
% 
% 
%     if(theta1 >= present_theta1)
%         %     if (present_theta1 > theta1_IK*maxerror || present_theta1 < theta1_IK*minerror)
%         %         a = 0;
%         %         theta1 = theta1 + abs(delta1)/count;
%         %     end
%         if (abs(present_theta1) <= (abs(theta1)+error) && abs(present_theta1) >= (abs(theta1)-error) && (sign(present_theta1) == sign(theta1_IK+2) || sign(present_theta1) == sign(theta1_IK-2)))
%             a = 1;
%             disp('a1')
%         else
%             if(abs(theta1-present_theta1)<(abs(delta1)/count1))
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
%         if (abs(present_theta1) <= (abs(theta1)+error) && abs(present_theta1) >= (abs(theta1)-error) && (sign(present_theta1) == sign(theta1_IK+2) || sign(present_theta1) == sign(theta1_IK-2)))
%             a = 1;
%             disp('a2')
%         else
%             if(abs(theta1-present_theta1)<(abs(delta1)/count1))
%                 count1 = count1*3;
%             end
%             a = 0;
%             theta1 = theta1 - abs(delta1)/count1;
%         end
%     end
% 
%     if(theta2 >= present_theta2)
%         if (abs(present_theta2) <= (abs(theta2)+error) && abs(present_theta3) >= (abs(theta2)-error) && (sign(present_theta2) == sign(theta2_IK+2) || sign(present_theta2) == sign(theta2_IK-2)))
%             b = 1;
%             disp('b1')
%         else
%             if(abs(theta2-present_theta2)<(abs(delta2)/count2))
%                 count2 = count2*3;
%             end
%             b = 0;
%             theta2 = theta2 + abs(delta2)/count2;
%         end
%     else
%         if (abs(present_theta2) <= (abs(theta2)+error) && abs(present_theta2) >= (abs(theta2)-error) && (sign(present_theta2) == sign(theta2_IK+2) || sign(present_theta2) == sign(theta2_IK-2)))
%             b = 1;
%             disp('b2')
%         else
%             if(abs(theta2-present_theta2)<(abs(delta2)/count2))
%                 count2 = count2*3;
%             end
%             b = 0;
%             theta2 = theta2 - abs(delta2)/count2;
%         end
%     end
% 
%     if(theta3 >= present_theta3)
%         if (abs(present_theta3) <= (abs(theta3)+error) && abs(present_theta3) >= (abs(theta3)-error) && (sign(present_theta3) == sign(theta3_IK+2) || sign(present_theta3) == sign(theta3_IK-2)))
%             c = 1;
%             disp('c1')
%         else
%             if(abs(theta3-present_theta3)<(abs(delta3)/count3))
%                 count3 = count3*3;
%             end
%             c = 0;
%             theta3 = theta3 + abs(delta3)/count3;
%         end
%     else
%         if (abs(present_theta3) <= (abs(theta3)+error) && abs(present_theta3) >= (abs(theta3)-error) && (sign(present_theta3) == sign(theta3_IK+2) || sign(present_theta3) == sign(theta3_IK-2)))
%             c = 1;
%             disp('c2')
%         else
%             if(abs(theta3-present_theta3)<(abs(delta3)/count3))
%                 count3 = count3*3;
%             end
%             c = 0;
%             theta3 = theta3 - abs(delta3)/count3;
%         end
%     end
% 
%     if(theta4 >= present_theta4)
%         if (abs(present_theta4) <= (abs(theta4)+error) && abs(present_theta4) >= (abs(theta4)-error) && (sign(present_theta4) == sign(theta4_IK+2) || sign(present_theta4) == sign(theta4_IK-2)))
%             d = 1;
%             disp('d1')
%         else
%             if(abs(theta4-present_theta4)<(abs(delta4)/count4))
%                 count4 = count4*3;
%             end
%             d = 0;
%             theta4 = theta4 + abs(delta4)/count4
%         end
%     else
%         if (abs(present_theta4) <= (abs(theta4)+error) && abs(present_theta4) >= (abs(theta4)-error) && (sign(present_theta4) == sign(theta4_IK+2) || sign(present_theta4) == sign(theta4_IK-2)))
%             d = 1;
%             disp('d2')
%         else
%             if(abs(theta4-present_theta4)<(abs(delta4)/count4))
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

if(gripper_pos == 1)
    encoder_theta4 = 225*2048/180;
end
if(gripper_pos == 0)
    encoder_theta4 = 1024;
end
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_PRO_GOAL_POSITION, encoder_theta4);
pause(0.5)

end