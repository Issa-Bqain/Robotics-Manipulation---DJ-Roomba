
% Read the position of the dynamixel horn with the torque off
% The code executes for a given amount of time then terminates


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
PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel

% Default setting
DXL_ID_1                      =11;           
DXL_ID_2                      =12;
DXL_ID_3                      =13;           
DXL_ID_4                      =14;
DXL_ID_5                      =15;           

BAUDRATE                    = 1000000;
DEVICENAME                  = 'COM8';       % Check which porheta
% is being used on your controller
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
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_PRO_OPERATING_MODE, 3);

% Disable Dynamixel Torque
%write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_TORQUE_ENABLE, 1);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_TORQUE_ENABLE, 1);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PRO_TORQUE_ENABLE, 1);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PRO_TORQUE_ENABLE, 1);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_PRO_TORQUE_ENABLE, 1);

% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_GOAL_POSITION, 2048);
%write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_GOAL_POSITION,700);

% 
%         dxl_present_position_1 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_PRESENT_POSITION)
%         dxl_present_position_2 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_PRESENT_POSITION)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

goalcoord=[17.4,10.5,0];%pos of target xz horizontal, y vertical

goalpose=-90;%angle of attack of tool with horizontal being zero, attack from below 90 etc

axis=[1,2];

%%%%%%%%%%%%%

L3=12.6;
L2=12.4;
L1=13;
%L12 is different

base2goal=atan2d(goalcoord(3),goalcoord(1));
%p1,p2,p3
p3=goalcoord;
p2=p3-[L3*cosd(goalpose),L3*sind(goalpose),L3*cosd(goalpose)*sind(base2goal)];

p2=p2-[0,7.7,0];

goalx=p2(1);
goaly=p2(2);

plot(goalcoord(axis(1)),goalcoord(axis(2)),'ro')

    
c2=(goalx^2+goaly^2-L1^2-L2^2)/(2*L1*L2);
s2up=sqrt(1-c2^2);
s2down=-sqrt(1-c2^2);

k1=L1+L2*c2;
k2up=L2*s2up;
k2down=L2*s2down;

theta2up=atan2(s2up,c2);
theta2down=atan2(s2down,c2);
theta1up=-atan2(goalx,goaly)-atan2(k2up,k1)+pi/2;%-atan2(128,24);
theta1down=-atan2(goalx,goaly)-atan2(k2down,k1)+pi/2;

base=[0,0,0];

node1=[0,7.7,0];

theta0=base2goal*pi/180;

node2=node1+[L1*cos(theta1down),L1*sin(theta1down),L1*tand(base2goal)*cos(theta1down)];

theta1=theta1down-atan2(12.8,2.4);

node3=node2+[L2*cos(theta1down+theta2down),L2*sin(theta1down+theta2down),L2*tand(base2goal)*cos(theta1down+theta2down)];

theta2=theta2down+theta1down;

node4=node3+[L3*cos(goalpose*pi/180),L3*sin(goalpose*pi/180),L3*tand(base2goal)*cos(goalpose*pi/180)];

theta3=goalpose*pi/180-theta2;




% Disable Dynamixel Torque
%write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
%write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
%dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
%dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
%if dxl_comm_result ~= COMM_SUCCESS
%    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
%elseif dxl_error ~= 0
%    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
%end

q=0;
prev=[27.4,20.5,0];
prevpose=0;
prevangle=[0,0,0,0];
figure;
coord1=[];
coord2=[];

write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_GOAL_POSITION, 2048);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_GOAL_POSITION, 2048);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PRO_GOAL_POSITION, 2048);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PRO_GOAL_POSITION, 2048);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_PRO_GOAL_POSITION, 1024);
% pause(1)
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_PRO_GOAL_POSITION, 216*2048/180);


while q~=1
   quit=input('Quit?\n');
   if(quit==1)
       q=1;
       break;
   end
   goalcoord=[input('Target X Position\n'),input('Target Y Position\n'),input('Target Z Position\n')]; 
   goalpose=input('Tool Pose\n');
   gripper_pos = input('Grip?\n');
   
   %change this%
    plane=[1,2];
    axis=plane;

    count=5;
    steps=15;
    maxerror=1.001;
    minerror=0.999;

    %%%%%%%%%%%%%

    L3=12.6;
    L2=12.4;
    L1=13;
    %L12 is different
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    intervalcoord=prev;
    intervalpose=prevpose;
    deltcoord=(goalcoord-intervalcoord)/steps;
    deltpose=(goalpose-intervalpose)/steps;
    

    
    for step=0:steps-1
        intervalcoord=intervalcoord+deltcoord;
        intervalpose=intervalpose+deltpose;
        
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_GOAL_POSITION, encoder_theta0);
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_GOAL_POSITION, encoder_theta1);
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PRO_GOAL_POSITION, encoder_theta2);
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PRO_GOAL_POSITION, encoder_theta3);
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_PRO_GOAL_POSITION, encoder_theta4);

     %   pause(0.1)


        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);

        if dxl_comm_result ~= COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        elseif dxl_error ~= 0
            fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
        else
            fprintf('Dynamixel has been successfully connected \n');
        end


        % --------------------------------------------------------------------------------------------------------------------------------

        % define the lines
        % all numbers are in centimeters

        theta1 = prevangle(1);
        theta2 = prevangle(2);
        theta3= prevangle(3);
        theta4= prevangle(4);
        a = 0;
        b = 0;
        c = 0;
        d = 0;
        length = 2; %the length of the coordinate axes


        %change these to set the goals for the final destination
        goal_theta1 = theta1_IK; 
        goal_theta2 = theta2_IK;
        goal_theta3 = theta3_IK;
        goal_theta4 = theta4_IK;

        if(plane(1) == 1 && plane(2) == 2)
            plane_c = 'X-Y';
        end
        if(plane(1) == 1 && plane(2) == 3)
            plane_c = 'X-Z';
        end
        if(plane(1) == 2 && plane(2) == 3)
            plane_c = 'Y-Z';
        end

        while (a==0 || b==0 || c==0 || d==0)

        xlim([-10 40])
        ylim([-10 40])

        % do the transformation matrices

           T_0=[cosd(theta1)          0      -sind(theta1)     0; 
                          0                1            0         2.8; %base offset
                      sind(theta1)         0      cosd(theta1)     0;
                          0                0            0           1] ;


            T_0_1 = [cosd(theta2)          -sind(theta2)        0           0; 
                      sind(theta2)          cosd(theta2)        0           4.9;
                      0                     0                   0           0
                      0                     0                   0           1] ;


            T_1_2 = [cosd(theta3)       -sind(theta3)       0           2.4 ;
                    sind(theta3)        cosd(theta3)        0            12.8; 
                          0                  0              1            0;
                          0                 0               0             1] ;

            T_2_3 = [cosd(theta4)           -sind(theta4)        0      12.4; 
                    sind(theta4)           cosd(theta4)          0       0; 
                       0                        0                1        0;
                       0                        0                0         1] ;

            T_3_4 = [ 1          0        0           14.6;
                      0          1        0              0;
                      0          0        1              0;
                      0          0        0              1];

          % get the joints coordinates 
        %   plane(1)
            joint1=T_0; 
            joint1x= joint1(plane(1),4);
            joint1y= joint1(plane(2),4);

          % coordinates for the axes

          if(plane(1) ==1 && plane(2) ==2)
            axis1joint1x_x = joint1(plane(1),4);
            axis1joint1y_x = joint1(plane(2),4);
            axis2joint1x_x = axis1joint1x_x + length*cosd(theta1);
            axis2joint1y_x = axis1joint1y_x;  % this only rotates in the z plane so the axis rotate around z

            axis1joint1x_y = joint1(plane(1),4);
            axis1joint1y_y = joint1(plane(2),4);
            axis2joint1x_y = axis1joint1x_x;
            axis2joint1y_y = axis1joint1y_x + length;


            joint2=T_0*T_0_1;
            joint2x=joint2(plane(1),4);
            joint2y=joint2(plane(2),4);
            axis1joint2x_x = joint2(plane(1),4);
            axis1joint2y_x = joint2(plane(2),4);
            axis2joint2x_x = axis1joint2x_x + length*cosd(theta2)*cosd(theta1); %cosdtheta1 rotates the axis around z 
            axis2joint2y_x = axis1joint2y_x + length*sind(theta2);

            axis1joint2x_y = joint2(plane(1),4);
            axis1joint2y_y = joint2(plane(2),4);
            axis2joint2x_y = axis1joint2x_x + length*cosd(theta2+90);
            axis2joint2y_y = axis1joint2y_x + length*sind(theta2+90);

            joint3=T_0*T_0_1*T_1_2;
            joint3x=joint3(plane(1),4);
            joint3y=joint3(plane(2),4);
            axis1joint3x_x = joint3(plane(1),4);
            axis1joint3y_x = joint3(plane(2),4);
            axis2joint3x_x = axis1joint3x_x + length*cosd(theta2+theta3)*cosd(theta1);
            axis2joint3y_x = axis1joint3y_x + length*sind(theta2+theta3);

            axis1joint3x_y = joint3(plane(1),4);
            axis1joint3y_y = joint3(plane(2),4);
            axis2joint3x_y = axis1joint3x_x + length*cosd(theta2+theta3+90);
            axis2joint3y_y = axis1joint3y_x + length*sind(theta2+theta3+90);

            joint4=T_0*T_0_1*T_1_2*T_2_3;
            joint4x=joint4(plane(1),4);
            joint4y=joint4(plane(2),4);
            axis1joint4x_x = joint4(plane(1),4);
            axis1joint4y_x = joint4(plane(2),4);
            axis2joint4x_x = axis1joint4x_x + length*cosd(theta2+theta3+theta4)*cosd(theta1);
            axis2joint4y_x = axis1joint4y_x + length*sind(theta2+theta3+theta4);

            axis1joint4x_y = joint4(plane(1),4);
            axis1joint4y_y = joint4(plane(2),4);
            axis2joint4x_y = axis1joint4x_x + length*cosd(theta2+theta3+theta4+90);
            axis2joint4y_y = axis1joint4y_x + length*sind(theta2+theta3+theta4+90);

            tool=T_0*T_0_1*T_1_2*T_2_3*T_3_4;
            toolx=tool(plane(1),4);
            tooly=tool(plane(2),4);
            axis1toolx_x = tool(plane(1),4);
            axis1tooly_x = tool(plane(2),4);
            axis2toolx_x = axis1toolx_x + length*cosd(theta2+theta3+theta4)*cosd(theta1);
            axis2tooly_x = axis1tooly_x + length*sind(theta2+theta3+theta4);

            axis1toolx_y = tool(plane(1),4);
            axis1tooly_y = tool(plane(2),4);
            axis2toolx_y = axis1toolx_x + length*cosd(theta2+theta3+theta4+90);
            axis2tooly_y = axis1tooly_x + length*sind(theta2+theta3+theta4+90);
          end

          if(plane(1) ==1 && plane(2) ==3)
            axis1joint1x_x = joint1(plane(1),4);
            axis1joint1y_x = joint1(plane(2),4);
            axis2joint1x_x = axis1joint1x_x + length*cosd(theta1);
            axis2joint1y_x = axis1joint1y_x + length*sind(theta1);  % this only rotates in the z plane so the axis rotate around z

            axis1joint1x_y = joint1(plane(1),4);
            axis1joint1y_y = joint1(plane(2),4);
            axis2joint1x_y = axis1joint1x_x + length*cosd(theta1+90);
            axis2joint1y_y = axis1joint1y_x + length*sind(theta1+90);


            joint2=T_0*T_0_1;
            joint2x=joint2(plane(1),4);
            joint2y=joint2(plane(2),4);
            axis1joint2x_x = joint2(plane(1),4);
            axis1joint2y_x = joint2(plane(2),4);
            axis2joint2x_x = axis1joint2x_x + length*cosd(theta1)*cosd(theta2); %cosdtheta1 rotates the axis around z 
            axis2joint2y_x = axis1joint2y_x + length*sind(theta1);

            axis1joint2x_y = joint2(plane(1),4);
            axis1joint2y_y = joint2(plane(2),4);
            axis2joint2x_y = axis1joint2x_x + length*cosd(theta1+90);
            axis2joint2y_y = axis1joint2y_x + length*sind(theta1+90);

            joint3=T_0*T_0_1*T_1_2;
            joint3x=joint3(plane(1),4);
            joint3y=joint3(plane(2),4);
            axis1joint3x_x = joint3(plane(1),4);
            axis1joint3y_x = joint3(plane(2),4);
            axis2joint3x_x = axis1joint3x_x + length*cosd(theta1)*cosd(theta2+theta3);
            axis2joint3y_x = axis1joint3y_x + length*sind(theta1);

            axis1joint3x_y = joint3(plane(1),4);
            axis1joint3y_y = joint3(plane(2),4);
            axis2joint3x_y = axis1joint3x_x + length*cosd(theta1+90);
            axis2joint3y_y = axis1joint3y_x + length*sind(theta1+90);

            joint4=T_0*T_0_1*T_1_2*T_2_3;
            joint4x=joint4(plane(1),4);
            joint4y=joint4(plane(2),4);
            axis1joint4x_x = joint4(plane(1),4);
            axis1joint4y_x = joint4(plane(2),4);
            axis2joint4x_x = axis1joint4x_x + length*cosd(theta1)*cosd(theta2+theta3+theta4);
            axis2joint4y_x = axis1joint4y_x + length*sind(theta1);

            axis1joint4x_y = joint4(plane(1),4);
            axis1joint4y_y = joint4(plane(2),4);
            axis2joint4x_y = axis1joint4x_x + length*cosd(theta1+90);
            axis2joint4y_y = axis1joint4y_x + length*sind(theta1+90);

            tool=T_0*T_0_1*T_1_2*T_2_3*T_3_4;
            toolx=tool(plane(1),4);
            tooly=tool(plane(2),4);
            axis1toolx_x = tool(plane(1),4);
            axis1tooly_x = tool(plane(2),4);
            axis2toolx_x = axis1toolx_x + length*cosd(theta1)*cosd(theta2+theta3+theta4);
            axis2tooly_x = axis1tooly_x + length*sind(theta1);

            axis1toolx_y = tool(plane(1),4);
            axis1tooly_y = tool(plane(2),4);
            axis2toolx_y = axis1toolx_x + length*cosd(theta1+90);
            axis2tooly_y = axis1tooly_x + length*sind(theta1+90);
          end

          if(plane(1) ==2 && plane(2) ==3)
            axis1joint1x_x = joint1(plane(1),4);
            axis1joint1y_x = joint1(plane(2),4);
            axis2joint1x_x = axis1joint1x_x + length;
            axis2joint1y_x = axis1joint1y_x;  % this only rotates in the z plane so the axis rotate around z

            axis1joint1x_y = joint1(plane(1),4);
            axis1joint1y_y = joint1(plane(2),4);
            axis2joint1x_y = axis1joint1x_x;
            axis2joint1y_y = axis1joint1y_x + length*cosd(theta1);


            joint2=T_0*T_0_1;
            joint2x=joint2(plane(1),4);
            joint2y=joint2(plane(2),4);
            axis1joint2x_x = joint2(plane(1),4);
            axis1joint2y_x = joint2(plane(2),4);
            axis2joint2x_x = axis1joint2x_x + length*cosd(theta2); %cosdtheta1 rotates the axis around z 
            axis2joint2y_x = axis1joint2y_x + length*sind(theta1);

            axis1joint2x_y = joint2(plane(1),4);
            axis1joint2y_y = joint2(plane(2),4);
            axis2joint2x_y = axis1joint2x_x;
            axis2joint2y_y = axis1joint2y_x + length*cosd(theta1);

            joint3=T_0*T_0_1*T_1_2;
            joint3x=joint3(plane(1),4);
            joint3y=joint3(plane(2),4);
            axis1joint3x_x = joint3(plane(1),4);
            axis1joint3y_x = joint3(plane(2),4);
            axis2joint3x_x = axis1joint3x_x + length*cosd(theta2+theta3);
            axis2joint3y_x = axis1joint3y_x + length*sind(theta1);

            axis1joint3x_y = joint3(plane(1),4);
            axis1joint3y_y = joint3(plane(2),4);
            axis2joint3x_y = axis1joint3x_x ;
            axis2joint3y_y = axis1joint3y_x + length*cosd(theta1);

            joint4=T_0*T_0_1*T_1_2*T_2_3;
            joint4x=joint4(plane(1),4);
            joint4y=joint4(plane(2),4);
            axis1joint4x_x = joint4(plane(1),4);
            axis1joint4y_x = joint4(plane(2),4);
            axis2joint4x_x = axis1joint4x_x + length*cosd(theta2+theta3+theta4);
            axis2joint4y_x = axis1joint4y_x + length*sind(theta1);

            axis1joint4x_y = joint4(plane(1),4);
            axis1joint4y_y = joint4(plane(2),4);
            axis2joint4x_y = axis1joint4x_x;
            axis2joint4y_y = axis1joint4y_x + length*cosd(theta1);

            tool=T_0*T_0_1*T_1_2*T_2_3*T_3_4;
            toolx=tool(plane(1),4);
            tooly=tool(plane(2),4);
            axis1toolx_x = tool(plane(1),4);
            axis1tooly_x = tool(plane(2),4);
            axis2toolx_x = axis1toolx_x + length*cosd(theta2+theta3+theta4);
            axis2tooly_x = axis1tooly_x + length*sind(theta1);

            axis1toolx_y = tool(plane(1),4);
            axis1tooly_y = tool(plane(2),4);
            axis2toolx_y = axis1toolx_x;
            axis2tooly_y = axis1tooly_x + length*cosd(theta1);
          end


         %coordinates

        j1_x = joint1x;
        j1_y = joint1y;
        x1 = [0 j1_x];
        y1 = [0 j1_y];

        j2_x = joint2x;
        j2_y = joint2y;
        x2 = [j1_x j2_x];
        y2 = [j1_y j2_y];

        j3_x = joint3x;
        j3_y = joint3y;
        x3 = [j2_x j3_x];
        y3 = [j2_y j3_y];

        j4_x = joint4x;
        j4_y = joint4y;
        x4 = [j3_x j4_x];
        y4 = [j3_y j4_y];

        tool_x = toolx;
        tool_y = tooly;
        xtool = [j4_x tool_x];
        ytool = [j4_y tool_y];
        hold on;
        %lines
        l1 = plot(x1, y1);
        l1.LineWidth = 2;
        l1.Color = 'blue';

        l2 = plot(x2, y2);
        l2.LineWidth = 2;
        l2.Color = 'black';

        l3 = plot(x3, y3, 'Color','#A2142F');
        l3.LineWidth = 2;
        %l3.Color = 'brown';

        l4 = plot(x4, y4, 'Color', '#7E2F8E');
        l4.LineWidth = 2;
        %l4.Color = 'purple';

        l5 = plot(xtool, ytool, 'Color', '#D95319');
        l5.LineWidth = 2;
        %l4.Color = 'orange';

        circle=plot(intervalcoord(axis(1)),intervalcoord(axis(2)),'ro');


         grid on
        title(['4DOF Simulation in the ' plane_c ' plane'])

        %lines for coordinates

        if(plane(1) ==1 && plane(2) ==2)

        axis_x1_joint1 = [axis1joint1x_x axis2joint1x_x];
        axis_x2_joint1 = [axis1joint1y_x axis2joint1y_x];
        axis_x_joint1 = line(axis_x1_joint1, axis_x2_joint1);
        %axis_x_elbow.Marker = '>';
        axis_x_joint1.Color = 'green';

        axis_y1_joint1 = [axis1joint1x_y axis2joint1x_y];
        axis_y2_joint1 = [axis1joint1y_y axis2joint1y_y];
        axis_y_joint1 = line(axis_y1_joint1, axis_y2_joint1);
        %axis_y_elbow.Marker = '>';
        axis_y_joint1.Color = 'blue';

        axis_x1_joint2 = [axis1joint2x_x axis2joint2x_x];
        axis_x2_joint2 = [axis1joint2y_x axis2joint2y_x];
        axis_x_joint2 = line(axis_x1_joint2, axis_x2_joint2);
        %axis_x_tool.Marker = '>';
        axis_x_joint2.Color = 'green';

        axis_y1_joint2 = [axis1joint2x_y axis2joint2x_y];
        axis_y2_joint2 = [axis1joint2y_y axis2joint2y_y];
        axis_y_joint2 = line(axis_y1_joint2, axis_y2_joint2);
        %axis_y_tool.Marker = '>';
        axis_y_joint2.Color = 'blue';

        axis_x1_joint3 = [axis1joint3x_x axis2joint3x_x];
        axis_x2_joint3 = [axis1joint3y_x axis2joint3y_x];
        axis_x_joint3 = line(axis_x1_joint3, axis_x2_joint3);
        %axis_x_tool.Marker = '>';
        axis_x_joint3.Color = 'green';

        axis_y1_joint3 = [axis1joint3x_y axis2joint3x_y];
        axis_y2_joint3 = [axis1joint3y_y axis2joint3y_y];
        axis_y_joint3 = line(axis_y1_joint3, axis_y2_joint3);
        %axis_y_tool.Marker = '>';
        axis_y_joint3.Color = 'blue';

        axis_x1_joint4 = [axis1joint4x_x axis2joint4x_x];
        axis_x2_joint4 = [axis1joint4y_x axis2joint4y_x];
        axis_x_joint4 = line(axis_x1_joint4, axis_x2_joint4);
        %axis_x_tool.Marker = '>';
        axis_x_joint4.Color = 'green';

        axis_y1_joint4 = [axis1joint4x_y axis2joint4x_y];
        axis_y2_joint4 = [axis1joint4y_y axis2joint4y_y];
        axis_y_joint4 = line(axis_y1_joint4, axis_y2_joint4);
        %axis_y_tool.Marker = '>';
        axis_y_joint4.Color = 'blue';

        axis_x1_tool = [axis1toolx_x axis2toolx_x];
        axis_x2_tool = [axis1tooly_x axis2tooly_x];
        axis_x_tool = line(axis_x1_tool, axis_x2_tool);
        %axis_x_tool.Marker = '>';
        axis_x_tool.Color = 'green';

        axis_y1_tool = [axis1toolx_y axis2toolx_y];
        axis_y2_tool = [axis1tooly_y axis2tooly_y];
        axis_y_tool = line(axis_y1_tool, axis_y2_tool);
        %axis_y_tool.Marker = '>';
        axis_y_tool.Color = 'blue';

        legend([axis_x_joint1 axis_y_joint1],{'X axis','Y axis'})
        end



        if(plane(1) ==1 && plane(2) ==3)

        axis_x1_joint1 = [axis1joint1x_x axis2joint1x_x];
        axis_x2_joint1 = [axis1joint1y_x axis2joint1y_x];
        axis_x_joint1 = line(axis_x1_joint1, axis_x2_joint1);
        %axis_x_elbow.Marker = '>';
        axis_x_joint1.Color = 'green';

        axis_y1_joint1 = [axis1joint1x_y axis2joint1x_y];
        axis_y2_joint1 = [axis1joint1y_y axis2joint1y_y];
        axis_y_joint1 = line(axis_y1_joint1, axis_y2_joint1);
        %axis_y_elbow.Marker = '>';
        axis_y_joint1.Color = 'red';

        axis_x1_joint2 = [axis1joint2x_x axis2joint2x_x];
        axis_x2_joint2 = [axis1joint2y_x axis2joint2y_x];
        axis_x_joint2 = line(axis_x1_joint2, axis_x2_joint2);
        %axis_x_tool.Marker = '>';
        axis_x_joint2.Color = 'green';

        axis_y1_joint2 = [axis1joint2x_y axis2joint2x_y];
        axis_y2_joint2 = [axis1joint2y_y axis2joint2y_y];
        axis_y_joint2 = line(axis_y1_joint2, axis_y2_joint2);
        %axis_y_tool.Marker = '>';
        axis_y_joint2.Color = 'red';

        axis_x1_joint3 = [axis1joint3x_x axis2joint3x_x];
        axis_x2_joint3 = [axis1joint3y_x axis2joint3y_x];
        axis_x_joint3 = line(axis_x1_joint3, axis_x2_joint3);
        %axis_x_tool.Marker = '>';
        axis_x_joint3.Color = 'green';

        axis_y1_joint3 = [axis1joint3x_y axis2joint3x_y];
        axis_y2_joint3 = [axis1joint3y_y axis2joint3y_y];
        axis_y_joint3 = line(axis_y1_joint3, axis_y2_joint3);
        %axis_y_tool.Marker = '>';
        axis_y_joint3.Color = 'red';

        axis_x1_joint4 = [axis1joint4x_x axis2joint4x_x];
        axis_x2_joint4 = [axis1joint4y_x axis2joint4y_x];
        axis_x_joint4 = line(axis_x1_joint4, axis_x2_joint4);
        %axis_x_tool.Marker = '>';
        axis_x_joint4.Color = 'green';

        axis_y1_joint4 = [axis1joint4x_y axis2joint4x_y];
        axis_y2_joint4 = [axis1joint4y_y axis2joint4y_y];
        axis_y_joint4 = line(axis_y1_joint4, axis_y2_joint4);
        %axis_y_tool.Marker = '>';
        axis_y_joint4.Color = 'red';

        axis_x1_tool = [axis1toolx_x axis2toolx_x];
        axis_x2_tool = [axis1tooly_x axis2tooly_x];
        axis_x_tool = line(axis_x1_tool, axis_x2_tool);
        %axis_x_tool.Marker = '>';
        axis_x_tool.Color = 'green';

        axis_y1_tool = [axis1toolx_y axis2toolx_y];
        axis_y2_tool = [axis1tooly_y axis2tooly_y];
        axis_y_tool = line(axis_y1_tool, axis_y2_tool);
        %axis_y_tool.Marker = '>';
        axis_y_tool.Color = 'red';

        legend([axis_x_joint1 axis_y_joint1],{'X axis','Z axis'})
        end

        if(plane(1) ==2 && plane(2) ==3)

        axis_x1_joint1 = [axis1joint1x_x axis2joint1x_x];
        axis_x2_joint1 = [axis1joint1y_x axis2joint1y_x];
        axis_x_joint1 = line(axis_x1_joint1, axis_x2_joint1);
        %axis_x_elbow.Marker = '>';
        axis_x_joint1.Color = 'blue';

        axis_y1_joint1 = [axis1joint1x_y axis2joint1x_y];
        axis_y2_joint1 = [axis1joint1y_y axis2joint1y_y];
        axis_y_joint1 = line(axis_y1_joint1, axis_y2_joint1);
        %axis_y_elbow.Marker = '>';
        axis_y_joint1.Color = 'red';

        axis_x1_joint2 = [axis1joint2x_x axis2joint2x_x];
        axis_x2_joint2 = [axis1joint2y_x axis2joint2y_x];
        axis_x_joint2 = line(axis_x1_joint2, axis_x2_joint2);
        %axis_x_tool.Marker = '>';
        axis_x_joint2.Color = 'blue';

        axis_y1_joint2 = [axis1joint2x_y axis2joint2x_y];
        axis_y2_joint2 = [axis1joint2y_y axis2joint2y_y];
        axis_y_joint2 = line(axis_y1_joint2, axis_y2_joint2);
        %axis_y_tool.Marker = '>';
        axis_y_joint2.Color = 'red';

        axis_x1_joint3 = [axis1joint3x_x axis2joint3x_x];
        axis_x2_joint3 = [axis1joint3y_x axis2joint3y_x];
        axis_x_joint3 = line(axis_x1_joint3, axis_x2_joint3);
        %axis_x_tool.Marker = '>';
        axis_x_joint3.Color = 'blue';

        axis_y1_joint3 = [axis1joint3x_y axis2joint3x_y];
        axis_y2_joint3 = [axis1joint3y_y axis2joint3y_y];
        axis_y_joint3 = line(axis_y1_joint3, axis_y2_joint3);
        %axis_y_tool.Marker = '>';
        axis_y_joint3.Color = 'red';

        axis_x1_joint4 = [axis1joint4x_x axis2joint4x_x];
        axis_x2_joint4 = [axis1joint4y_x axis2joint4y_x];
        axis_x_joint4 = line(axis_x1_joint4, axis_x2_joint4);
        %axis_x_tool.Marker = '>';
        axis_x_joint4.Color = 'blue';

        axis_y1_joint4 = [axis1joint4x_y axis2joint4x_y];
        axis_y2_joint4 = [axis1joint4y_y axis2joint4y_y];
        axis_y_joint4 = line(axis_y1_joint4, axis_y2_joint4);
        %axis_y_tool.Marker = '>';
        axis_y_joint4.Color = 'red';

        axis_x1_tool = [axis1toolx_x axis2toolx_x];
        axis_x2_tool = [axis1tooly_x axis2tooly_x];
        axis_x_tool = line(axis_x1_tool, axis_x2_tool);
        %axis_x_tool.Marker = '>';
        axis_x_tool.Color = 'blue';

        axis_y1_tool = [axis1toolx_y axis2toolx_y];
        axis_y2_tool = [axis1tooly_y axis2tooly_y];
        axis_y_tool = line(axis_y1_tool, axis_y2_tool);
        %axis_y_tool.Marker = '>';
        axis_y_tool.Color = 'red';

        legend([axis_x_joint1 axis_y_joint1],{'Y axis','Z axis'})
        end

        pres1=-(180/2048)*(read4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID_1,ADDR_PRO_PRESENT_POSITION)-2048)
        pres2=-(180/2048)*(read4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID_2,ADDR_PRO_PRESENT_POSITION)-2048)
        pres3=-(180/2048)*(read4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID_3,ADDR_PRO_PRESENT_POSITION)-2048)
        pres4=-(180/2048)*(read4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID_4,ADDR_PRO_PRESENT_POSITION)-2048)
        
%                 encoder_theta0 = -(theta1_IK*2048 )/ 180 +2048;

        
        delta1=goal_theta1-pres1;%prevangle(1);
        delta2=goal_theta2-pres2;%prevangle(2);
        delta3=goal_theta3-pres3;%prevangle(3);
        delta4=goal_theta4-pres4;%prevangle(4);

        % how many iterations, how fast the simulation goes and when to stop
        

        if(goal_theta1 >= theta1)
            if (pres1 > goal_theta1*maxerror || pres1 < goal_theta1*minerror)
                a = 0;
                theta1 = theta1 + abs(delta1)/count;
            end
            if (abs(pres1) <= abs(goal_theta1)*maxerror && abs(pres1) >= abs(goal_theta1)*minerror && sign(pres1) == sign(goal_theta1))
                a = 1;
                disp('a1')
            end
        end
        if(goal_theta1 < pres1)
          if (pres1 > goal_theta1*maxerror || pres1 < goal_theta1*minerror)
            a = 0;
            theta1 = theta1 - abs(delta1)/count;
          end
          if (abs(pres1) <= abs(goal_theta1)*maxerror && abs(pres1) >= abs(goal_theta1)*minerror && sign(pres1) == sign(goal_theta1))
            a = 1;
            disp('a2')
          end  
        end

        if(goal_theta2 >= pres2)
            if (pres2 > (goal_theta2 + 0.01)*maxerror || pres2 < (goal_theta2-0.01)*minerror)
                b = 0;
                theta2 = theta2 + abs(delta2)/count;
                
            end
            if (abs(pres2) <= (abs(goal_theta2)+0.01)*maxerror && abs(pres2) >= (abs(goal_theta2)-0.01)*minerror && (sign(pres2) == sign(goal_theta2+0.01) || sign(pres2) == sign(goal_theta2-0.01)))
                b = 1;
                disp('b1')
            end
        end
        if(goal_theta2 < pres2)
            if (pres2 > (goal_theta2+0.01)*maxerror || pres2 < (goal_theta2-0.01)*minerror)
                b = 0;
                theta2 = theta2 - abs(delta2)/count;
                
            end
            if (abs(pres2) <= (abs(goal_theta2)+0.01)*maxerror && abs(pres2) >= (abs(goal_theta2)-0.01)*minerror && (sign(pres2) == sign(goal_theta2+0.01) || sign(pres2) == sign(goal_theta2-0.01)))
                b = 1;
                disp('b2')
            end
        end

        if(goal_theta3 >= pres3)
            if (pres3 > goal_theta3*maxerror || pres3 < goal_theta3*minerror)
                c = 0;
                theta3 = theta3 + abs(delta3)/count;
            end
            if (abs(pres3) <= abs(goal_theta3)*maxerror && abs(pres3) >= abs(goal_theta3)*minerror && sign(pres3) == sign(goal_theta3))
                c = 1;
                disp('c1')
            end
        end
        if(goal_theta3 < pres3)
            if (pres3 > goal_theta3*maxerror || pres3 < goal_theta3*minerror)
                c = 0;
                theta3 = theta3 - abs(delta3)/count;
            end
            if (abs(pres3) <= abs(goal_theta3)*maxerror && abs(pres3) >= abs(goal_theta3)*minerror && sign(pres3) == sign(goal_theta3))
                c = 1;
                disp('c2')
            end 
        end

        if(goal_theta4 >= pres4)
            if (pres4 > goal_theta4*maxerror || pres4 < goal_theta4*minerror)
                d = 0;
                theta4 = theta4 + abs(delta4)/count;
            end
            if (abs(pres4) <= abs(goal_theta4)*maxerror && abs(pres4) >= abs(goal_theta4)*minerror && sign(pres4) == sign(goal_theta4))
                d = 1;
                disp('d1')
            end
        end
        if(goal_theta4 < pres4)
            if (pres4 > goal_theta4*maxerror || pres4 < goal_theta4*minerror)
                d = 0;
                theta4 = theta4 - abs(delta4)/count;
            end
            if (abs(pres4) <= abs(goal_theta4)*maxerror && abs(pres4) >= abs(goal_theta4)*minerror && sign(pres4) == sign(goal_theta4))
                d = 1;
                disp('d2')
            end
        end

        entheta0 = -(theta1*2048 )/ 180 +2048;
        entheta1 = -(theta2*2048 )/ 180 +2048 ;
        entheta2 = -(theta3*2048 )/ 180 +2048;
        entheta3 = -(theta4*2048 )/ 180 +2048;

        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_GOAL_POSITION, entheta0);
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_GOAL_POSITION, entheta1);
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PRO_GOAL_POSITION, entheta2);
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PRO_GOAL_POSITION, entheta3);
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_5, ADDR_PRO_GOAL_POSITION, encoder_theta4);


        % need to clear figures so it looks like a video
        coord1(end+1)=xtool(2);
        coord2(end+1)=ytool(2);
        plot(coord1,coord2,'b')
        
    %    pause(0.01)

        clf
        hold off;
        
        end
    

        

        prev=intervalcoord;
        prevangle=[theta1,theta2,theta3,theta4];
        prevpose=intervalpose;
        hold off;
        
        a=1
    end
    % at the end the figure is cleared so needs plotting outside while loop
    xlim([-10 40])
    ylim([-10 40])

    % do the transformation matrices

    T_0=[cosd(theta1)          0      -sind(theta1)     0; 
                  0                1            0         2.8; %base offset
              sind(theta1)         0      cosd(theta1)     0;
                  0                0            0           1] ;


    T_0_1 = [cosd(theta2)          -sind(theta2)        0           0; 
              sind(theta2)          cosd(theta2)        0           4.9;
              0                     0                   0           0
              0                     0                   0           1] ;


    T_1_2 = [cosd(theta3)       -sind(theta3)       0           2.4 ;
            sind(theta3)        cosd(theta3)        0            12.8; 
                  0                  0              1            0;
                  0                 0               0             1] ;

    T_2_3 = [cosd(theta4)           -sind(theta4)        0      12.4; 
            sind(theta4)           cosd(theta4)          0       0; 
               0                        0                1        0;
               0                        0                0         1] ;

    T_3_4 = [ 1          0        0           14.6;
              0          1        0              0;
              0          0        1              0;
              0          0        0              1];

      % get the joints coordinates 
    %   plane(1)
        joint1=T_0; 
        joint1x= joint1(plane(1),4);
        joint1y= joint1(plane(2),4);

      % coordinates for the axes

      if(plane(1) ==1 && plane(2) ==2)
        axis1joint1x_x = joint1(plane(1),4);
        axis1joint1y_x = joint1(plane(2),4);
        axis2joint1x_x = axis1joint1x_x + length*cosd(theta1);
        axis2joint1y_x = axis1joint1y_x;  % this only rotates in the z plane so the axis rotate around z

        axis1joint1x_y = joint1(plane(1),4);
        axis1joint1y_y = joint1(plane(2),4);
        axis2joint1x_y = axis1joint1x_x;
        axis2joint1y_y = axis1joint1y_x + length;


        joint2=T_0*T_0_1;
        joint2x=joint2(plane(1),4);
        joint2y=joint2(plane(2),4);
        axis1joint2x_x = joint2(plane(1),4);
        axis1joint2y_x = joint2(plane(2),4);
        axis2joint2x_x = axis1joint2x_x + length*cosd(theta2)*cosd(theta1); %cosdtheta1 rotates the axis around z 
        axis2joint2y_x = axis1joint2y_x + length*sind(theta2);

        axis1joint2x_y = joint2(plane(1),4);
        axis1joint2y_y = joint2(plane(2),4);
        axis2joint2x_y = axis1joint2x_x + length*cosd(theta2+90);
        axis2joint2y_y = axis1joint2y_x + length*sind(theta2+90);

        joint3=T_0*T_0_1*T_1_2;
        joint3x=joint3(plane(1),4);
        joint3y=joint3(plane(2),4);
        axis1joint3x_x = joint3(plane(1),4);
        axis1joint3y_x = joint3(plane(2),4);
        axis2joint3x_x = axis1joint3x_x + length*cosd(theta2+theta3)*cosd(theta1);
        axis2joint3y_x = axis1joint3y_x + length*sind(theta2+theta3);

        axis1joint3x_y = joint3(plane(1),4);
        axis1joint3y_y = joint3(plane(2),4);
        axis2joint3x_y = axis1joint3x_x + length*cosd(theta2+theta3+90);
        axis2joint3y_y = axis1joint3y_x + length*sind(theta2+theta3+90);

        joint4=T_0*T_0_1*T_1_2*T_2_3;
        joint4x=joint4(plane(1),4);
        joint4y=joint4(plane(2),4);
        axis1joint4x_x = joint4(plane(1),4);
        axis1joint4y_x = joint4(plane(2),4);
        axis2joint4x_x = axis1joint4x_x + length*cosd(theta2+theta3+theta4)*cosd(theta1);
        axis2joint4y_x = axis1joint4y_x + length*sind(theta2+theta3+theta4);

        axis1joint4x_y = joint4(plane(1),4);
        axis1joint4y_y = joint4(plane(2),4);
        axis2joint4x_y = axis1joint4x_x + length*cosd(theta2+theta3+theta4+90);
        axis2joint4y_y = axis1joint4y_x + length*sind(theta2+theta3+theta4+90);

        tool=T_0*T_0_1*T_1_2*T_2_3*T_3_4;
        toolx=tool(plane(1),4);
        tooly=tool(plane(2),4);
        axis1toolx_x = tool(plane(1),4);
        axis1tooly_x = tool(plane(2),4);
        axis2toolx_x = axis1toolx_x + length*cosd(theta2+theta3+theta4)*cosd(theta1);
        axis2tooly_x = axis1tooly_x + length*sind(theta2+theta3+theta4);

        axis1toolx_y = tool(plane(1),4);
        axis1tooly_y = tool(plane(2),4);
        axis2toolx_y = axis1toolx_x + length*cosd(theta2+theta3+theta4+90);
        axis2tooly_y = axis1tooly_x + length*sind(theta2+theta3+theta4+90);
      end

      if(plane(1) ==1 && plane(2) ==3)
        axis1joint1x_x = joint1(plane(1),4);
        axis1joint1y_x = joint1(plane(2),4);
        axis2joint1x_x = axis1joint1x_x + length*cosd(theta1);
        axis2joint1y_x = axis1joint1y_x + length*sind(theta1);  % this only rotates in the z plane so the axis rotate around z

        axis1joint1x_y = joint1(plane(1),4);
        axis1joint1y_y = joint1(plane(2),4);
        axis2joint1x_y = axis1joint1x_x + length*cosd(theta1+90);
        axis2joint1y_y = axis1joint1y_x + length*sind(theta1+90);


        joint2=T_0*T_0_1;
        joint2x=joint2(plane(1),4);
        joint2y=joint2(plane(2),4);
        axis1joint2x_x = joint2(plane(1),4);
        axis1joint2y_x = joint2(plane(2),4);
        axis2joint2x_x = axis1joint2x_x + length*cosd(theta1)*cosd(theta2); %cosdtheta1 rotates the axis around z 
        axis2joint2y_x = axis1joint2y_x + length*sind(theta1);

        axis1joint2x_y = joint2(plane(1),4);
        axis1joint2y_y = joint2(plane(2),4);
        axis2joint2x_y = axis1joint2x_x + length*cosd(theta1+90);
        axis2joint2y_y = axis1joint2y_x + length*sind(theta1+90);

        joint3=T_0*T_0_1*T_1_2;
        joint3x=joint3(plane(1),4);
        joint3y=joint3(plane(2),4);
        axis1joint3x_x = joint3(plane(1),4);
        axis1joint3y_x = joint3(plane(2),4);
        axis2joint3x_x = axis1joint3x_x + length*cosd(theta1)*cosd(theta2+theta3);
        axis2joint3y_x = axis1joint3y_x + length*sind(theta1);

        axis1joint3x_y = joint3(plane(1),4);
        axis1joint3y_y = joint3(plane(2),4);
        axis2joint3x_y = axis1joint3x_x + length*cosd(theta1+90);
        axis2joint3y_y = axis1joint3y_x + length*sind(theta1+90);

        joint4=T_0*T_0_1*T_1_2*T_2_3;
        joint4x=joint4(plane(1),4);
        joint4y=joint4(plane(2),4);
        axis1joint4x_x = joint4(plane(1),4);
        axis1joint4y_x = joint4(plane(2),4);
        axis2joint4x_x = axis1joint4x_x + length*cosd(theta1)*cosd(theta2+theta3+theta4);
        axis2joint4y_x = axis1joint4y_x + length*sind(theta1);

        axis1joint4x_y = joint4(plane(1),4);
        axis1joint4y_y = joint4(plane(2),4);
        axis2joint4x_y = axis1joint4x_x + length*cosd(theta1+90);
        axis2joint4y_y = axis1joint4y_x + length*sind(theta1+90);

        tool=T_0*T_0_1*T_1_2*T_2_3*T_3_4;
        toolx=tool(plane(1),4);
        tooly=tool(plane(2),4);
        axis1toolx_x = tool(plane(1),4);
        axis1tooly_x = tool(plane(2),4);
        axis2toolx_x = axis1toolx_x + length*cosd(theta1)*cosd(theta2+theta3+theta4);
        axis2tooly_x = axis1tooly_x + length*sind(theta1);

        axis1toolx_y = tool(plane(1),4);
        axis1tooly_y = tool(plane(2),4);
        axis2toolx_y = axis1toolx_x + length*cosd(theta1+90);
        axis2tooly_y = axis1tooly_x + length*sind(theta1+90);
      end

      if(plane(1) ==2 && plane(2) ==3)
        axis1joint1x_x = joint1(plane(1),4);
        axis1joint1y_x = joint1(plane(2),4);
        axis2joint1x_x = axis1joint1x_x + length;
        axis2joint1y_x = axis1joint1y_x;  % this only rotates in the z plane so the axis rotate around z

        axis1joint1x_y = joint1(plane(1),4);
        axis1joint1y_y = joint1(plane(2),4);
        axis2joint1x_y = axis1joint1x_x;
        axis2joint1y_y = axis1joint1y_x + length*cosd(theta1);


        joint2=T_0*T_0_1;
        joint2x=joint2(plane(1),4);
        joint2y=joint2(plane(2),4);
        axis1joint2x_x = joint2(plane(1),4);
        axis1joint2y_x = joint2(plane(2),4);
        axis2joint2x_x = axis1joint2x_x + length*cosd(theta2); %cosdtheta1 rotates the axis around z 
        axis2joint2y_x = axis1joint2y_x + length*sind(theta1);

        axis1joint2x_y = joint2(plane(1),4);
        axis1joint2y_y = joint2(plane(2),4);
        axis2joint2x_y = axis1joint2x_x;
        axis2joint2y_y = axis1joint2y_x + length*cosd(theta1);

        joint3=T_0*T_0_1*T_1_2;
        joint3x=joint3(plane(1),4);
        joint3y=joint3(plane(2),4);
        axis1joint3x_x = joint3(plane(1),4);
        axis1joint3y_x = joint3(plane(2),4);
        axis2joint3x_x = axis1joint3x_x + length*cosd(theta2+theta3);
        axis2joint3y_x = axis1joint3y_x + length*sind(theta1);

        axis1joint3x_y = joint3(plane(1),4);
        axis1joint3y_y = joint3(plane(2),4);
        axis2joint3x_y = axis1joint3x_x ;
        axis2joint3y_y = axis1joint3y_x + length*cosd(theta1);

        joint4=T_0*T_0_1*T_1_2*T_2_3;
        joint4x=joint4(plane(1),4);
        joint4y=joint4(plane(2),4);
        axis1joint4x_x = joint4(plane(1),4);
        axis1joint4y_x = joint4(plane(2),4);
        axis2joint4x_x = axis1joint4x_x + length*cosd(theta2+theta3+theta4);
        axis2joint4y_x = axis1joint4y_x + length*sind(theta1);

        axis1joint4x_y = joint4(plane(1),4);
        axis1joint4y_y = joint4(plane(2),4);
        axis2joint4x_y = axis1joint4x_x;
        axis2joint4y_y = axis1joint4y_x + length*cosd(theta1);

        tool=T_0*T_0_1*T_1_2*T_2_3*T_3_4;
        toolx=tool(plane(1),4);
        tooly=tool(plane(2),4);
        axis1toolx_x = tool(plane(1),4);
        axis1tooly_x = tool(plane(2),4);
        axis2toolx_x = axis1toolx_x + length*cosd(theta2+theta3+theta4);
        axis2tooly_x = axis1tooly_x + length*sind(theta1);

        axis1toolx_y = tool(plane(1),4);
        axis1tooly_y = tool(plane(2),4);
        axis2toolx_y = axis1toolx_x;
        axis2tooly_y = axis1tooly_x + length*cosd(theta1);
      end


     %coordinates

    j1_x = joint1x;
    j1_y = joint1y;
    x1 = [0 j1_x];
    y1 = [0 j1_y];

    j2_x = joint2x;
    j2_y = joint2y;
    x2 = [j1_x j2_x];
    y2 = [j1_y j2_y];

    j3_x = joint3x;
    j3_y = joint3y;
    x3 = [j2_x j3_x];
    y3 = [j2_y j3_y];

    j4_x = joint4x;
    j4_y = joint4y;
    x4 = [j3_x j4_x];
    y4 = [j3_y j4_y];

    tool_x = toolx;
    tool_y = tooly;
    xtool = [j4_x tool_x];
    ytool = [j4_y tool_y];
    hold on
    l1 = line(x1, y1);
    l1.Color = 'blue';
    l1.LineWidth = 2;
    l2 = line(x2, y2);
    l2.Color = 'black';
    l2.LineWidth = 2;
    l3 = line(x3, y3, 'Color','#A2142F' );
    l3.LineWidth = 2;
    %l3.Color = 'brown';
    l4 = line(x4, y4, 'Color','#7E2F8E' );
    l4.LineWidth = 2;
    %l4.Color = 'purple';
    l5 = line(xtool, ytool, 'Color', '#D95319');
    l5.LineWidth = 2;
    %l4.Color = 'orange';



    grid on
    title(['4DOF Simulation in the ' plane_c ' plane'])


    if(plane(1) ==1 && plane(2) ==2)

    axis_x1_joint1 = [axis1joint1x_x axis2joint1x_x];
    axis_x2_joint1 = [axis1joint1y_x axis2joint1y_x];
    axis_x_joint1 = line(axis_x1_joint1, axis_x2_joint1);
    %axis_x_elbow.Marker = '>';
    axis_x_joint1.Color = 'green';

    axis_y1_joint1 = [axis1joint1x_y axis2joint1x_y];
    axis_y2_joint1 = [axis1joint1y_y axis2joint1y_y];
    axis_y_joint1 = line(axis_y1_joint1, axis_y2_joint1);
    %axis_y_elbow.Marker = '>';
    axis_y_joint1.Color = 'blue';

    axis_x1_joint2 = [axis1joint2x_x axis2joint2x_x];
    axis_x2_joint2 = [axis1joint2y_x axis2joint2y_x];
    axis_x_joint2 = line(axis_x1_joint2, axis_x2_joint2);
    %axis_x_tool.Marker = '>';
    axis_x_joint2.Color = 'green';

    axis_y1_joint2 = [axis1joint2x_y axis2joint2x_y];
    axis_y2_joint2 = [axis1joint2y_y axis2joint2y_y];
    axis_y_joint2 = line(axis_y1_joint2, axis_y2_joint2);
    %axis_y_tool.Marker = '>';
    axis_y_joint2.Color = 'blue';

    axis_x1_joint3 = [axis1joint3x_x axis2joint3x_x];
    axis_x2_joint3 = [axis1joint3y_x axis2joint3y_x];
    axis_x_joint3 = line(axis_x1_joint3, axis_x2_joint3);
    %axis_x_tool.Marker = '>';
    axis_x_joint3.Color = 'green';

    axis_y1_joint3 = [axis1joint3x_y axis2joint3x_y];
    axis_y2_joint3 = [axis1joint3y_y axis2joint3y_y];
    axis_y_joint3 = line(axis_y1_joint3, axis_y2_joint3);
    %axis_y_tool.Marker = '>';
    axis_y_joint3.Color = 'blue';

    axis_x1_joint4 = [axis1joint4x_x axis2joint4x_x];
    axis_x2_joint4 = [axis1joint4y_x axis2joint4y_x];
    axis_x_joint4 = line(axis_x1_joint4, axis_x2_joint4);
    %axis_x_tool.Marker = '>';
    axis_x_joint4.Color = 'green';

    axis_y1_joint4 = [axis1joint4x_y axis2joint4x_y];
    axis_y2_joint4 = [axis1joint4y_y axis2joint4y_y];
    axis_y_joint4 = line(axis_y1_joint4, axis_y2_joint4);
    %axis_y_tool.Marker = '>';
    axis_y_joint4.Color = 'blue';

    axis_x1_tool = [axis1toolx_x axis2toolx_x];
    axis_x2_tool = [axis1tooly_x axis2tooly_x];
    axis_x_tool = line(axis_x1_tool, axis_x2_tool);
    %axis_x_tool.Marker = '>';
    axis_x_tool.Color = 'green';

    axis_y1_tool = [axis1toolx_y axis2toolx_y];
    axis_y2_tool = [axis1tooly_y axis2tooly_y];
    axis_y_tool = line(axis_y1_tool, axis_y2_tool);
    %axis_y_tool.Marker = '>';
    axis_y_tool.Color = 'blue';

    legend([axis_x_joint1 axis_y_joint1],{'X axis','Y axis'})
    end


    if(plane(1) ==1 && plane(2) ==3)

    axis_x1_joint1 = [axis1joint1x_x axis2joint1x_x];
    axis_x2_joint1 = [axis1joint1y_x axis2joint1y_x];
    axis_x_joint1 = line(axis_x1_joint1, axis_x2_joint1);
    %axis_x_elbow.Marker = '>';
    axis_x_joint1.Color = 'green';

    axis_y1_joint1 = [axis1joint1x_y axis2joint1x_y];
    axis_y2_joint1 = [axis1joint1y_y axis2joint1y_y];
    axis_y_joint1 = line(axis_y1_joint1, axis_y2_joint1);
    %axis_y_elbow.Marker = '>';
    axis_y_joint1.Color = 'red';

    axis_x1_joint2 = [axis1joint2x_x axis2joint2x_x];
    axis_x2_joint2 = [axis1joint2y_x axis2joint2y_x];
    axis_x_joint2 = line(axis_x1_joint2, axis_x2_joint2);
    %axis_x_tool.Marker = '>';
    axis_x_joint2.Color = 'green';

    axis_y1_joint2 = [axis1joint2x_y axis2joint2x_y];
    axis_y2_joint2 = [axis1joint2y_y axis2joint2y_y];
    axis_y_joint2 = line(axis_y1_joint2, axis_y2_joint2);
    %axis_y_tool.Marker = '>';
    axis_y_joint2.Color = 'red';

    axis_x1_joint3 = [axis1joint3x_x axis2joint3x_x];
    axis_x2_joint3 = [axis1joint3y_x axis2joint3y_x];
    axis_x_joint3 = line(axis_x1_joint3, axis_x2_joint3);
    %axis_x_tool.Marker = '>';
    axis_x_joint3.Color = 'green';

    axis_y1_joint3 = [axis1joint3x_y axis2joint3x_y];
    axis_y2_joint3 = [axis1joint3y_y axis2joint3y_y];
    axis_y_joint3 = line(axis_y1_joint3, axis_y2_joint3);
    %axis_y_tool.Marker = '>';
    axis_y_joint3.Color = 'red';

    axis_x1_joint4 = [axis1joint4x_x axis2joint4x_x];
    axis_x2_joint4 = [axis1joint4y_x axis2joint4y_x];
    axis_x_joint4 = line(axis_x1_joint4, axis_x2_joint4);
    %axis_x_tool.Marker = '>';
    axis_x_joint4.Color = 'green';

    axis_y1_joint4 = [axis1joint4x_y axis2joint4x_y];
    axis_y2_joint4 = [axis1joint4y_y axis2joint4y_y];
    axis_y_joint4 = line(axis_y1_joint4, axis_y2_joint4);
    %axis_y_tool.Marker = '>';
    axis_y_joint4.Color = 'red';

    axis_x1_tool = [axis1toolx_x axis2toolx_x];
    axis_x2_tool = [axis1tooly_x axis2tooly_x];
    axis_x_tool = line(axis_x1_tool, axis_x2_tool);
    %axis_x_tool.Marker = '>';
    axis_x_tool.Color = 'green';

    axis_y1_tool = [axis1toolx_y axis2toolx_y];
    axis_y2_tool = [axis1tooly_y axis2tooly_y];
    axis_y_tool = line(axis_y1_tool, axis_y2_tool);
    %axis_y_tool.Marker = '>';
    axis_y_tool.Color = 'red';

    legend([axis_x_joint1 axis_y_joint1],{'X axis','Z axis'})
    end

    if(plane(1) ==2 && plane(2) ==3)

    axis_x1_joint1 = [axis1joint1x_x axis2joint1x_x];
    axis_x2_joint1 = [axis1joint1y_x axis2joint1y_x];
    axis_x_joint1 = line(axis_x1_joint1, axis_x2_joint1);
    %axis_x_elbow.Marker = '>';
    axis_x_joint1.Color = 'blue';

    axis_y1_joint1 = [axis1joint1x_y axis2joint1x_y];
    axis_y2_joint1 = [axis1joint1y_y axis2joint1y_y];
    axis_y_joint1 = line(axis_y1_joint1, axis_y2_joint1);
    %axis_y_elbow.Marker = '>';
    axis_y_joint1.Color = 'red';

    axis_x1_joint2 = [axis1joint2x_x axis2joint2x_x];
    axis_x2_joint2 = [axis1joint2y_x axis2joint2y_x];
    axis_x_joint2 = line(axis_x1_joint2, axis_x2_joint2);
    %axis_x_tool.Marker = '>';
    axis_x_joint2.Color = 'blue';

    axis_y1_joint2 = [axis1joint2x_y axis2joint2x_y];
    axis_y2_joint2 = [axis1joint2y_y axis2joint2y_y];
    axis_y_joint2 = line(axis_y1_joint2, axis_y2_joint2);
    %axis_y_tool.Marker = '>';
    axis_y_joint2.Color = 'red';

    axis_x1_joint3 = [axis1joint3x_x axis2joint3x_x];
    axis_x2_joint3 = [axis1joint3y_x axis2joint3y_x];
    axis_x_joint3 = line(axis_x1_joint3, axis_x2_joint3);
    %axis_x_tool.Marker = '>';
    axis_x_joint3.Color = 'blue';

    axis_y1_joint3 = [axis1joint3x_y axis2joint3x_y];
    axis_y2_joint3 = [axis1joint3y_y axis2joint3y_y];
    axis_y_joint3 = line(axis_y1_joint3, axis_y2_joint3);
    %axis_y_tool.Marker = '>';
    axis_y_joint3.Color = 'red';

    axis_x1_joint4 = [axis1joint4x_x axis2joint4x_x];
    axis_x2_joint4 = [axis1joint4y_x axis2joint4y_x];
    axis_x_joint4 = line(axis_x1_joint4, axis_x2_joint4);
    %axis_x_tool.Marker = '>';
    axis_x_joint4.Color = 'blue';

    axis_y1_joint4 = [axis1joint4x_y axis2joint4x_y];
    axis_y2_joint4 = [axis1joint4y_y axis2joint4y_y];
    axis_y_joint4 = line(axis_y1_joint4, axis_y2_joint4);
    %axis_y_tool.Marker = '>';
    axis_y_joint4.Color = 'red';

    axis_x1_tool = [axis1toolx_x axis2toolx_x];
    axis_x2_tool = [axis1tooly_x axis2tooly_x];
    axis_x_tool = line(axis_x1_tool, axis_x2_tool);
    %axis_x_tool.Marker = '>';
    axis_x_tool.Color = 'blue';

    axis_y1_tool = [axis1toolx_y axis2toolx_y];
    axis_y2_tool = [axis1tooly_y axis2tooly_y];
    axis_y_tool = line(axis_y1_tool, axis_y2_tool);
    %axis_y_tool.Marker = '>';
    axis_y_tool.Color = 'red';

    legend([axis_x_joint1 axis_y_joint1],{'Y axis','Z axis'})
    end

    coord1(end+1)=xtool(2);
    coord2(end+1)=ytool(2);
    plot(coord1,coord2,'b')
    
    xlim([-10 40])
    ylim([-10 40])
    hold off;
end

% at the end the figure is cleared so needs plotting outside while loop
xlim([-10 40])
ylim([-10 40])

% do the transformation matrices

T_0=[cosd(theta1)          0      -sind(theta1)     0; 
              0                1            0         2.8; %base offset
          sind(theta1)         0      cosd(theta1)     0;
              0                0            0           1] ;


T_0_1 = [cosd(theta2)          -sind(theta2)        0           0; 
          sind(theta2)          cosd(theta2)        0           4.9;
          0                     0                   0           0
          0                     0                   0           1] ;


T_1_2 = [cosd(theta3)       -sind(theta3)       0           2.4 ;
        sind(theta3)        cosd(theta3)        0            12.8; 
              0                  0              1            0;
              0                 0               0             1] ;

T_2_3 = [cosd(theta4)           -sind(theta4)        0      12.4; 
        sind(theta4)           cosd(theta4)          0       0; 
           0                        0                1        0;
           0                        0                0         1] ;

T_3_4 = [ 1          0        0           14.6;
          0          1        0              0;
          0          0        1              0;
          0          0        0              1];

  % get the joints coordinates 
%   plane(1)
    joint1=T_0; 
    joint1x= joint1(plane(1),4);
    joint1y= joint1(plane(2),4);

  % coordinates for the axes

  if(plane(1) ==1 && plane(2) ==2)
    axis1joint1x_x = joint1(plane(1),4);
    axis1joint1y_x = joint1(plane(2),4);
    axis2joint1x_x = axis1joint1x_x + length*cosd(theta1);
    axis2joint1y_x = axis1joint1y_x;  % this only rotates in the z plane so the axis rotate around z

    axis1joint1x_y = joint1(plane(1),4);
    axis1joint1y_y = joint1(plane(2),4);
    axis2joint1x_y = axis1joint1x_x;
    axis2joint1y_y = axis1joint1y_x + length;


    joint2=T_0*T_0_1;
    joint2x=joint2(plane(1),4);
    joint2y=joint2(plane(2),4);
    axis1joint2x_x = joint2(plane(1),4);
    axis1joint2y_x = joint2(plane(2),4);
    axis2joint2x_x = axis1joint2x_x + length*cosd(theta2)*cosd(theta1); %cosdtheta1 rotates the axis around z 
    axis2joint2y_x = axis1joint2y_x + length*sind(theta2);

    axis1joint2x_y = joint2(plane(1),4);
    axis1joint2y_y = joint2(plane(2),4);
    axis2joint2x_y = axis1joint2x_x + length*cosd(theta2+90);
    axis2joint2y_y = axis1joint2y_x + length*sind(theta2+90);

    joint3=T_0*T_0_1*T_1_2;
    joint3x=joint3(plane(1),4);
    joint3y=joint3(plane(2),4);
    axis1joint3x_x = joint3(plane(1),4);
    axis1joint3y_x = joint3(plane(2),4);
    axis2joint3x_x = axis1joint3x_x + length*cosd(theta2+theta3)*cosd(theta1);
    axis2joint3y_x = axis1joint3y_x + length*sind(theta2+theta3);

    axis1joint3x_y = joint3(plane(1),4);
    axis1joint3y_y = joint3(plane(2),4);
    axis2joint3x_y = axis1joint3x_x + length*cosd(theta2+theta3+90);
    axis2joint3y_y = axis1joint3y_x + length*sind(theta2+theta3+90);

    joint4=T_0*T_0_1*T_1_2*T_2_3;
    joint4x=joint4(plane(1),4);
    joint4y=joint4(plane(2),4);
    axis1joint4x_x = joint4(plane(1),4);
    axis1joint4y_x = joint4(plane(2),4);
    axis2joint4x_x = axis1joint4x_x + length*cosd(theta2+theta3+theta4)*cosd(theta1);
    axis2joint4y_x = axis1joint4y_x + length*sind(theta2+theta3+theta4);

    axis1joint4x_y = joint4(plane(1),4);
    axis1joint4y_y = joint4(plane(2),4);
    axis2joint4x_y = axis1joint4x_x + length*cosd(theta2+theta3+theta4+90);
    axis2joint4y_y = axis1joint4y_x + length*sind(theta2+theta3+theta4+90);

    tool=T_0*T_0_1*T_1_2*T_2_3*T_3_4;
    toolx=tool(plane(1),4);
    tooly=tool(plane(2),4);
    axis1toolx_x = tool(plane(1),4);
    axis1tooly_x = tool(plane(2),4);
    axis2toolx_x = axis1toolx_x + length*cosd(theta2+theta3+theta4)*cosd(theta1);
    axis2tooly_x = axis1tooly_x + length*sind(theta2+theta3+theta4);

    axis1toolx_y = tool(plane(1),4);
    axis1tooly_y = tool(plane(2),4);
    axis2toolx_y = axis1toolx_x + length*cosd(theta2+theta3+theta4+90);
    axis2tooly_y = axis1tooly_x + length*sind(theta2+theta3+theta4+90);
  end

  if(plane(1) ==1 && plane(2) ==3)
    axis1joint1x_x = joint1(plane(1),4);
    axis1joint1y_x = joint1(plane(2),4);
    axis2joint1x_x = axis1joint1x_x + length*cosd(theta1);
    axis2joint1y_x = axis1joint1y_x + length*sind(theta1);  % this only rotates in the z plane so the axis rotate around z

    axis1joint1x_y = joint1(plane(1),4);
    axis1joint1y_y = joint1(plane(2),4);
    axis2joint1x_y = axis1joint1x_x + length*cosd(theta1+90);
    axis2joint1y_y = axis1joint1y_x + length*sind(theta1+90);


    joint2=T_0*T_0_1;
    joint2x=joint2(plane(1),4);
    joint2y=joint2(plane(2),4);
    axis1joint2x_x = joint2(plane(1),4);
    axis1joint2y_x = joint2(plane(2),4);
    axis2joint2x_x = axis1joint2x_x + length*cosd(theta1)*cosd(theta2); %cosdtheta1 rotates the axis around z 
    axis2joint2y_x = axis1joint2y_x + length*sind(theta1);

    axis1joint2x_y = joint2(plane(1),4);
    axis1joint2y_y = joint2(plane(2),4);
    axis2joint2x_y = axis1joint2x_x + length*cosd(theta1+90);
    axis2joint2y_y = axis1joint2y_x + length*sind(theta1+90);

    joint3=T_0*T_0_1*T_1_2;
    joint3x=joint3(plane(1),4);
    joint3y=joint3(plane(2),4);
    axis1joint3x_x = joint3(plane(1),4);
    axis1joint3y_x = joint3(plane(2),4);
    axis2joint3x_x = axis1joint3x_x + length*cosd(theta1)*cosd(theta2+theta3);
    axis2joint3y_x = axis1joint3y_x + length*sind(theta1);

    axis1joint3x_y = joint3(plane(1),4);
    axis1joint3y_y = joint3(plane(2),4);
    axis2joint3x_y = axis1joint3x_x + length*cosd(theta1+90);
    axis2joint3y_y = axis1joint3y_x + length*sind(theta1+90);

    joint4=T_0*T_0_1*T_1_2*T_2_3;
    joint4x=joint4(plane(1),4);
    joint4y=joint4(plane(2),4);
    axis1joint4x_x = joint4(plane(1),4);
    axis1joint4y_x = joint4(plane(2),4);
    axis2joint4x_x = axis1joint4x_x + length*cosd(theta1)*cosd(theta2+theta3+theta4);
    axis2joint4y_x = axis1joint4y_x + length*sind(theta1);

    axis1joint4x_y = joint4(plane(1),4);
    axis1joint4y_y = joint4(plane(2),4);
    axis2joint4x_y = axis1joint4x_x + length*cosd(theta1+90);
    axis2joint4y_y = axis1joint4y_x + length*sind(theta1+90);

    tool=T_0*T_0_1*T_1_2*T_2_3*T_3_4;
    toolx=tool(plane(1),4);
    tooly=tool(plane(2),4);
    axis1toolx_x = tool(plane(1),4);
    axis1tooly_x = tool(plane(2),4);
    axis2toolx_x = axis1toolx_x + length*cosd(theta1)*cosd(theta2+theta3+theta4);
    axis2tooly_x = axis1tooly_x + length*sind(theta1);

    axis1toolx_y = tool(plane(1),4);
    axis1tooly_y = tool(plane(2),4);
    axis2toolx_y = axis1toolx_x + length*cosd(theta1+90);
    axis2tooly_y = axis1tooly_x + length*sind(theta1+90);
  end

  if(plane(1) ==2 && plane(2) ==3)
    axis1joint1x_x = joint1(plane(1),4);
    axis1joint1y_x = joint1(plane(2),4);
    axis2joint1x_x = axis1joint1x_x + length;
    axis2joint1y_x = axis1joint1y_x;  % this only rotates in the z plane so the axis rotate around z

    axis1joint1x_y = joint1(plane(1),4);
    axis1joint1y_y = joint1(plane(2),4);
    axis2joint1x_y = axis1joint1x_x;
    axis2joint1y_y = axis1joint1y_x + length*cosd(theta1);


    joint2=T_0*T_0_1;
    joint2x=joint2(plane(1),4);
    joint2y=joint2(plane(2),4);
    axis1joint2x_x = joint2(plane(1),4);
    axis1joint2y_x = joint2(plane(2),4);
    axis2joint2x_x = axis1joint2x_x + length*cosd(theta2); %cosdtheta1 rotates the axis around z 
    axis2joint2y_x = axis1joint2y_x + length*sind(theta1);

    axis1joint2x_y = joint2(plane(1),4);
    axis1joint2y_y = joint2(plane(2),4);
    axis2joint2x_y = axis1joint2x_x;
    axis2joint2y_y = axis1joint2y_x + length*cosd(theta1);

    joint3=T_0*T_0_1*T_1_2;
    joint3x=joint3(plane(1),4);
    joint3y=joint3(plane(2),4);
    axis1joint3x_x = joint3(plane(1),4);
    axis1joint3y_x = joint3(plane(2),4);
    axis2joint3x_x = axis1joint3x_x + length*cosd(theta2+theta3);
    axis2joint3y_x = axis1joint3y_x + length*sind(theta1);

    axis1joint3x_y = joint3(plane(1),4);
    axis1joint3y_y = joint3(plane(2),4);
    axis2joint3x_y = axis1joint3x_x ;
    axis2joint3y_y = axis1joint3y_x + length*cosd(theta1);

    joint4=T_0*T_0_1*T_1_2*T_2_3;
    joint4x=joint4(plane(1),4);
    joint4y=joint4(plane(2),4);
    axis1joint4x_x = joint4(plane(1),4);
    axis1joint4y_x = joint4(plane(2),4);
    axis2joint4x_x = axis1joint4x_x + length*cosd(theta2+theta3+theta4);
    axis2joint4y_x = axis1joint4y_x + length*sind(theta1);

    axis1joint4x_y = joint4(plane(1),4);
    axis1joint4y_y = joint4(plane(2),4);
    axis2joint4x_y = axis1joint4x_x;
    axis2joint4y_y = axis1joint4y_x + length*cosd(theta1);

    tool=T_0*T_0_1*T_1_2*T_2_3*T_3_4;
    toolx=tool(plane(1),4);
    tooly=tool(plane(2),4);
    axis1toolx_x = tool(plane(1),4);
    axis1tooly_x = tool(plane(2),4);
    axis2toolx_x = axis1toolx_x + length*cosd(theta2+theta3+theta4);
    axis2tooly_x = axis1tooly_x + length*sind(theta1);

    axis1toolx_y = tool(plane(1),4);
    axis1tooly_y = tool(plane(2),4);
    axis2toolx_y = axis1toolx_x;
    axis2tooly_y = axis1tooly_x + length*cosd(theta1);
  end


 %coordinates

j1_x = joint1x;
j1_y = joint1y;
x1 = [0 j1_x];
y1 = [0 j1_y];

j2_x = joint2x;
j2_y = joint2y;
x2 = [j1_x j2_x];
y2 = [j1_y j2_y];

j3_x = joint3x;
j3_y = joint3y;
x3 = [j2_x j3_x];
y3 = [j2_y j3_y];

j4_x = joint4x;
j4_y = joint4y;
x4 = [j3_x j4_x];
y4 = [j3_y j4_y];

tool_x = toolx;
tool_y = tooly;
xtool = [j4_x tool_x];
ytool = [j4_y tool_y];

l1 = line(x1, y1);
l1.Color = 'blue';
l1.LineWidth = 2;
l2 = line(x2, y2);
l2.Color = 'black';
l2.LineWidth = 2;
l3 = line(x3, y3, 'Color','#A2142F' );
l3.LineWidth = 2;
%l3.Color = 'brown';
l4 = line(x4, y4, 'Color','#7E2F8E' );
l4.LineWidth = 2;
%l4.Color = 'purple';
l5 = line(xtool, ytool, 'Color', '#D95319');
l5.LineWidth = 2;
%l4.Color = 'orange';



grid on
title(['4DOF Simulation in the ' plane_c ' plane'])


if(plane(1) ==1 && plane(2) ==2)

axis_x1_joint1 = [axis1joint1x_x axis2joint1x_x];
axis_x2_joint1 = [axis1joint1y_x axis2joint1y_x];
axis_x_joint1 = line(axis_x1_joint1, axis_x2_joint1);
%axis_x_elbow.Marker = '>';
axis_x_joint1.Color = 'green';

axis_y1_joint1 = [axis1joint1x_y axis2joint1x_y];
axis_y2_joint1 = [axis1joint1y_y axis2joint1y_y];
axis_y_joint1 = line(axis_y1_joint1, axis_y2_joint1);
%axis_y_elbow.Marker = '>';
axis_y_joint1.Color = 'blue';

axis_x1_joint2 = [axis1joint2x_x axis2joint2x_x];
axis_x2_joint2 = [axis1joint2y_x axis2joint2y_x];
axis_x_joint2 = line(axis_x1_joint2, axis_x2_joint2);
%axis_x_tool.Marker = '>';
axis_x_joint2.Color = 'green';

axis_y1_joint2 = [axis1joint2x_y axis2joint2x_y];
axis_y2_joint2 = [axis1joint2y_y axis2joint2y_y];
axis_y_joint2 = line(axis_y1_joint2, axis_y2_joint2);
%axis_y_tool.Marker = '>';
axis_y_joint2.Color = 'blue';

axis_x1_joint3 = [axis1joint3x_x axis2joint3x_x];
axis_x2_joint3 = [axis1joint3y_x axis2joint3y_x];
axis_x_joint3 = line(axis_x1_joint3, axis_x2_joint3);
%axis_x_tool.Marker = '>';
axis_x_joint3.Color = 'green';

axis_y1_joint3 = [axis1joint3x_y axis2joint3x_y];
axis_y2_joint3 = [axis1joint3y_y axis2joint3y_y];
axis_y_joint3 = line(axis_y1_joint3, axis_y2_joint3);
%axis_y_tool.Marker = '>';
axis_y_joint3.Color = 'blue';

axis_x1_joint4 = [axis1joint4x_x axis2joint4x_x];
axis_x2_joint4 = [axis1joint4y_x axis2joint4y_x];
axis_x_joint4 = line(axis_x1_joint4, axis_x2_joint4);
%axis_x_tool.Marker = '>';
axis_x_joint4.Color = 'green';

axis_y1_joint4 = [axis1joint4x_y axis2joint4x_y];
axis_y2_joint4 = [axis1joint4y_y axis2joint4y_y];
axis_y_joint4 = line(axis_y1_joint4, axis_y2_joint4);
%axis_y_tool.Marker = '>';
axis_y_joint4.Color = 'blue';

axis_x1_tool = [axis1toolx_x axis2toolx_x];
axis_x2_tool = [axis1tooly_x axis2tooly_x];
axis_x_tool = line(axis_x1_tool, axis_x2_tool);
%axis_x_tool.Marker = '>';
axis_x_tool.Color = 'green';

axis_y1_tool = [axis1toolx_y axis2toolx_y];
axis_y2_tool = [axis1tooly_y axis2tooly_y];
axis_y_tool = line(axis_y1_tool, axis_y2_tool);
%axis_y_tool.Marker = '>';
axis_y_tool.Color = 'blue';

legend([axis_x_joint1 axis_y_joint1],{'X axis','Y axis'})
end


if(plane(1) ==1 && plane(2) ==3)

axis_x1_joint1 = [axis1joint1x_x axis2joint1x_x];
axis_x2_joint1 = [axis1joint1y_x axis2joint1y_x];
axis_x_joint1 = line(axis_x1_joint1, axis_x2_joint1);
%axis_x_elbow.Marker = '>';
axis_x_joint1.Color = 'green';

axis_y1_joint1 = [axis1joint1x_y axis2joint1x_y];
axis_y2_joint1 = [axis1joint1y_y axis2joint1y_y];
axis_y_joint1 = line(axis_y1_joint1, axis_y2_joint1);
%axis_y_elbow.Marker = '>';
axis_y_joint1.Color = 'red';

axis_x1_joint2 = [axis1joint2x_x axis2joint2x_x];
axis_x2_joint2 = [axis1joint2y_x axis2joint2y_x];
axis_x_joint2 = line(axis_x1_joint2, axis_x2_joint2);
%axis_x_tool.Marker = '>';
axis_x_joint2.Color = 'green';

axis_y1_joint2 = [axis1joint2x_y axis2joint2x_y];
axis_y2_joint2 = [axis1joint2y_y axis2joint2y_y];
axis_y_joint2 = line(axis_y1_joint2, axis_y2_joint2);
%axis_y_tool.Marker = '>';
axis_y_joint2.Color = 'red';

axis_x1_joint3 = [axis1joint3x_x axis2joint3x_x];
axis_x2_joint3 = [axis1joint3y_x axis2joint3y_x];
axis_x_joint3 = line(axis_x1_joint3, axis_x2_joint3);
%axis_x_tool.Marker = '>';
axis_x_joint3.Color = 'green';

axis_y1_joint3 = [axis1joint3x_y axis2joint3x_y];
axis_y2_joint3 = [axis1joint3y_y axis2joint3y_y];
axis_y_joint3 = line(axis_y1_joint3, axis_y2_joint3);
%axis_y_tool.Marker = '>';
axis_y_joint3.Color = 'red';

axis_x1_joint4 = [axis1joint4x_x axis2joint4x_x];
axis_x2_joint4 = [axis1joint4y_x axis2joint4y_x];
axis_x_joint4 = line(axis_x1_joint4, axis_x2_joint4);
%axis_x_tool.Marker = '>';
axis_x_joint4.Color = 'green';

axis_y1_joint4 = [axis1joint4x_y axis2joint4x_y];
axis_y2_joint4 = [axis1joint4y_y axis2joint4y_y];
axis_y_joint4 = line(axis_y1_joint4, axis_y2_joint4);
%axis_y_tool.Marker = '>';
axis_y_joint4.Color = 'red';

axis_x1_tool = [axis1toolx_x axis2toolx_x];
axis_x2_tool = [axis1tooly_x axis2tooly_x];
axis_x_tool = line(axis_x1_tool, axis_x2_tool);
%axis_x_tool.Marker = '>';
axis_x_tool.Color = 'green';

axis_y1_tool = [axis1toolx_y axis2toolx_y];
axis_y2_tool = [axis1tooly_y axis2tooly_y];
axis_y_tool = line(axis_y1_tool, axis_y2_tool);
%axis_y_tool.Marker = '>';
axis_y_tool.Color = 'red';

legend([axis_x_joint1 axis_y_joint1],{'X axis','Z axis'})
end

if(plane(1) ==2 && plane(2) ==3)

axis_x1_joint1 = [axis1joint1x_x axis2joint1x_x];
axis_x2_joint1 = [axis1joint1y_x axis2joint1y_x];
axis_x_joint1 = line(axis_x1_joint1, axis_x2_joint1);
%axis_x_elbow.Marker = '>';
axis_x_joint1.Color = 'blue';

axis_y1_joint1 = [axis1joint1x_y axis2joint1x_y];
axis_y2_joint1 = [axis1joint1y_y axis2joint1y_y];
axis_y_joint1 = line(axis_y1_joint1, axis_y2_joint1);
%axis_y_elbow.Marker = '>';
axis_y_joint1.Color = 'red';

axis_x1_joint2 = [axis1joint2x_x axis2joint2x_x];
axis_x2_joint2 = [axis1joint2y_x axis2joint2y_x];
axis_x_joint2 = line(axis_x1_joint2, axis_x2_joint2);
%axis_x_tool.Marker = '>';
axis_x_joint2.Color = 'blue';

axis_y1_joint2 = [axis1joint2x_y axis2joint2x_y];
axis_y2_joint2 = [axis1joint2y_y axis2joint2y_y];
axis_y_joint2 = line(axis_y1_joint2, axis_y2_joint2);
%axis_y_tool.Marker = '>';
axis_y_joint2.Color = 'red';

axis_x1_joint3 = [axis1joint3x_x axis2joint3x_x];
axis_x2_joint3 = [axis1joint3y_x axis2joint3y_x];
axis_x_joint3 = line(axis_x1_joint3, axis_x2_joint3);
%axis_x_tool.Marker = '>';
axis_x_joint3.Color = 'blue';

axis_y1_joint3 = [axis1joint3x_y axis2joint3x_y];
axis_y2_joint3 = [axis1joint3y_y axis2joint3y_y];
axis_y_joint3 = line(axis_y1_joint3, axis_y2_joint3);
%axis_y_tool.Marker = '>';
axis_y_joint3.Color = 'red';

axis_x1_joint4 = [axis1joint4x_x axis2joint4x_x];
axis_x2_joint4 = [axis1joint4y_x axis2joint4y_x];
axis_x_joint4 = line(axis_x1_joint4, axis_x2_joint4);
%axis_x_tool.Marker = '>';
axis_x_joint4.Color = 'blue';

axis_y1_joint4 = [axis1joint4x_y axis2joint4x_y];
axis_y2_joint4 = [axis1joint4y_y axis2joint4y_y];
axis_y_joint4 = line(axis_y1_joint4, axis_y2_joint4);
%axis_y_tool.Marker = '>';
axis_y_joint4.Color = 'red';

axis_x1_tool = [axis1toolx_x axis2toolx_x];
axis_x2_tool = [axis1tooly_x axis2tooly_x];
axis_x_tool = line(axis_x1_tool, axis_x2_tool);
%axis_x_tool.Marker = '>';
axis_x_tool.Color = 'blue';

axis_y1_tool = [axis1toolx_y axis2toolx_y];
axis_y2_tool = [axis1tooly_y axis2tooly_y];
axis_y_tool = line(axis_y1_tool, axis_y2_tool);
%axis_y_tool.Marker = '>';
axis_y_tool.Color = 'red';

legend([axis_x_joint1 axis_y_joint1],{'Y axis','Z axis'})
end

coord1(end+1)=xtool(2);
coord2(end+1)=ytool(2);
plot(coord1,coord2,'b')


% Close port
closePort(port_num);
fprintf('Port Closed \n');

% Unload Library
unloadlibrary(lib_name);

%close all;
%clear all;
