close all;
clear all;
q=0;

prev=[27.4,20.5,0];
prevpose=0;
prevangle=[0,0,0,0];
figure;
coord1=[];
coord2=[];
while q~=1
   quit=input('Quit?\n');
   if(quit==1)
       q=1;
       break;
   end
   goalcoord=[input('Target X Position\n'),input('Target Y Position\n'),input('Target Z Position\n')]; 
   goalpose=input('Tool Pose\n');
   
   %change this%
    plane=[1,2];
    axis=plane;

    count=2;
    steps=10;
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
        step
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

        theta1_IK = theta1_IK *180/pi;
        theta2_IK = theta2_IK *180/pi;
        theta3_IK = theta3_IK *180/pi;
        theta4_IK = theta4_IK *180/pi;


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

            T_3_4 = [ 1          0        0           12.6;
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


        delta1=goal_theta1-theta1;
        delta2=goal_theta2-theta2;
        delta3=goal_theta3-theta3;
        delta4=goal_theta4-theta4;

        % how many iterations, how fast the simulation goes and when to stop
        if(goal_theta1 >= theta1)
            if (theta1 > goal_theta1*maxerror || theta1 < goal_theta1*minerror)
                a = 0;
                theta1 = theta1 + abs(delta1)/count;
            end
            if (abs(theta1) <= abs(goal_theta1)*maxerror && abs(theta1) >= abs(goal_theta1)*minerror && sign(theta1) == sign(goal_theta1))
                a = 1;
                disp('a1')
            end
        end
        if(goal_theta1 < theta1)
          if (theta1 > goal_theta1*maxerror || theta1 < goal_theta1*minerror)
            a = 0;
            theta1 = theta1 - abs(delta1)/count;
          end
          if (abs(theta1) <= abs(goal_theta1)*maxerror && abs(theta1) >= abs(goal_theta1)*minerror && sign(theta1) == sign(goal_theta1))
            a = 1;
            disp('a2')
          end  
        end

        if(goal_theta2 >= theta2)
            if (theta2 > (goal_theta2 + 0.01)*maxerror || theta2 < (goal_theta2-0.01)*minerror)
                b = 0;
                theta2 = theta2 + abs(delta2)/count;
                theta2
            end
            if (abs(theta2) <= (abs(goal_theta2)+0.01)*maxerror && abs(theta2) >= (abs(goal_theta2)-0.01)*minerror && (sign(theta2) == sign(goal_theta2+0.01) || sign(theta2) == sign(goal_theta2-0.01)))
                b = 1;
                disp('b1')
            end
        end
        if(goal_theta2 < theta2)
            if (theta2 > (goal_theta2+0.01)*maxerror || theta2 < (goal_theta2-0.01)*minerror)
                b = 0;
                theta2 = theta2 - abs(delta2)/count;
                theta2
            end
            if (abs(theta2) <= (abs(goal_theta2)+0.01)*maxerror && abs(theta2) >= (abs(goal_theta2)-0.01)*minerror && (sign(theta2) == sign(goal_theta2+0.01) || sign(theta2) == sign(goal_theta2-0.01)))
                b = 1;
                disp('b2')
            end
        end

        if(goal_theta3 >= theta3)
            if (theta3 > goal_theta3*maxerror || theta3 < goal_theta3*minerror)
                c = 0;
                theta3 = theta3 + abs(delta3)/count;
            end
            if (abs(theta3) <= abs(goal_theta3)*maxerror && abs(theta3) >= abs(goal_theta3)*minerror && sign(theta3) == sign(goal_theta3))
                c = 1;
                disp('c1')
            end
        end
        if(goal_theta3 < theta3)
            if (theta3 > goal_theta3*maxerror || theta3 < goal_theta3*minerror)
                c = 0;
                theta3 = theta3 - abs(delta3)/count;
            end
            if (abs(theta3) <= abs(goal_theta3)*maxerror && abs(theta3) >= abs(goal_theta3)*minerror && sign(theta3) == sign(goal_theta3))
                c = 1;
                disp('c2')
            end 
        end

        if(goal_theta4 >= theta4)
            if (theta4 > goal_theta4*maxerror || theta4 < goal_theta4*minerror)
                d = 0;
                theta4 = theta4 + abs(delta4)/count;
            end
            if (abs(theta4) <= abs(goal_theta4)*maxerror && abs(theta4) >= abs(goal_theta4)*minerror && sign(theta4) == sign(goal_theta4))
                d = 1;
                disp('d1')
            end
        end
        if(goal_theta4 < theta4)
            if (theta4 > goal_theta4*maxerror || theta4 < goal_theta4*minerror)
                d = 0;
                theta4 = theta4 - abs(delta4)/count;
            end
            if (abs(theta4) <= abs(goal_theta4)*maxerror && abs(theta4) >= abs(goal_theta4)*minerror && sign(theta4) == sign(goal_theta4))
                d = 1;
                disp('d2')
            end
        end

        % need to clear figures so it looks like a video
        coord1(end+1)=xtool(2);
        coord2(end+1)=ytool(2);
        plot(coord1,coord2,'b')
        
        pause(0.01)

        clf
        hold off;
        
        end
    

        

        prev=intervalcoord;
        prevangle=[theta1,theta2,theta3,theta4];
        prevpose=intervalpose;
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

    T_3_4 = [ 1          0        0           12.6;
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

T_3_4 = [ 1          0        0           12.6;
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



    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    






