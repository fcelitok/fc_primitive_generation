%% In this program we are calculating primitive for given Ackermann vehicles
%
%function is working with max_steering_angle, car wheelbase and resolution

clear all;
close all;

steering_angle = 30;
wheelbase = 0.65;
resolution = 0.05;

numberOfAngle = 16;
max_dist = 5.0; %tested up tp 2.0m
numberOfIntermediatePose = 10;

steering_angle_max = steering_angle*pi/180; % in radiants
min_radius = wheelbase/tan(steering_angle_max);

long_distance_factor = 10; % 10 times make bigger direct primitives

forward_penalty = 1; % forward primitives
long_forward_penalty = 1;


print_stuff=1;

if(print_stuff==1)
    figure();
    hold on;
    grid on;
end

primID = 1;

%% Simple distance calculate

start_position = [0 0];
for_loop_search = [resolution:resolution:max_dist];

angles = [atan2(0,1) atan2(1,2) atan2(1,1) atan2(2,1) atan2(1,0) atan2(2,-1) atan2(1,-1) atan2(1,-2) atan2(0,-1) atan2(-1,-2) atan2(-1,-1) atan2(-2,-1) atan2(-1,0) atan2(-2,1) atan2(-1,1) atan2(-1,2)];

% atan2(0,1) direction 0
% atan2(1,2  direction 1
% atan2(1,1)  direction 2
% atan2(2,1) direction 3
% atan2(1 0)  direction 4
% atan(2,-1) direction 5
% atan2(1, -1) direction 6
% atan2(1,-2) direction 7
% atan2(0, -1) direction 8
% atan2(-1,-2) direction 9
% atan2(-1 -1)  direction 10
% atan2(-2,-1)  direction 11
% atan2(-1,0) direction 12
% atan2(-2,1) direction 13
% atan2(-1,1) direction 14
% atan2(-1,2) direction 15

%% Straight lines
k = 2;
y = [0:k-1 k*ones(1,2*k+1) k-1:-1:-k+1 -k*ones(1,2*k+1) -k+1:-1];
x = -[-k*ones(1,k) [-k:k-1] k*ones(1,2*k+1) k-1:-1:-k -k*ones(1,k-1)];

degree = 1;

while degree <= 16
    if (degree == 2 || degree == 4 || degree == 6 || degree == 8 || degree == 10 || degree == 12 || degree == 14 || degree == 16)
        plot(x(degree)*resolution, y(degree)*resolution, 'g*')

        straight_start_pose_theta = angles(degree);
        straight_start_pose = [start_position straight_start_pose_theta];

        end_position = [x(degree)*resolution y(degree)*resolution];
        straight_end_pose_theta = straight_start_pose_theta;
        straight_end_pose = [end_position straight_end_pose_theta];
        
        end_position_long = [x(degree)*resolution*long_distance_factor y(degree)*resolution*long_distance_factor];
        straight_end_pose_long = [end_position_long straight_end_pose_theta];

        [clotoid_check_straight, p_straight, L ] = clotoide(straight_start_pose, straight_end_pose, numberOfIntermediatePose, min_radius);
        [clotoid_check_straight_long, p_straight_long, L ] = clotoide(straight_start_pose, straight_end_pose_long, numberOfIntermediatePose, min_radius);

        primitives(primID).real_start_pose = straight_start_pose;
        primitives(primID).real_end_pose = straight_end_pose;
        primitives(primID).start_orientation = degree-1;
        prim_end_pose = [straight_end_pose(1)/resolution straight_end_pose(2)/resolution find(angles==straight_end_pose(3))-1];
        primitives(primID).end_pose = prim_end_pose;
        primitives(primID).intermediate_poses = p_straight;
        primitives(primID).additionalActionCostMult = forward_penalty;
        primID = primID + 1;
        
        primitives(primID).real_start_pose = straight_start_pose;
        primitives(primID).real_end_pose = straight_end_pose_long;
        primitives(primID).intermediate_poses = p_straight_long;
        primitives(primID).additionalActionCostMult = long_forward_penalty;
        primitives(primID).start_orientation = degree-1;
        prim_end_pose_long = [straight_end_pose_long(1)/resolution straight_end_pose_long(2)/resolution find(angles==straight_end_pose_long(3))-1];
        primitives(primID).end_pose = prim_end_pose_long;
        primID = primID + 1;

        if(print_stuff==1)
            plot(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),'+-r');
            plot(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),'+-g');
            grid on;
            hold on;
            quiver(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),...
                cos(primitives(primID-2).intermediate_poses(:,3)),sin(primitives(primID-2).intermediate_poses(:,3)),0.1,...
                'Color',[0 0 0]);
            quiver(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),...
                cos(primitives(primID-1).intermediate_poses(:,3)),sin(primitives(primID-1).intermediate_poses(:,3)),0.1,...
                'Color',[0 0 0]);
            pause(0.1);
        end
        degree = degree + 1;
    else
        plot(x(degree)*(resolution/2), y(degree)*(resolution/2), 'g*')

        straight_start_pose_theta = angles(degree);
        straight_start_pose = [start_position straight_start_pose_theta];

        end_position = [x(degree)*(resolution/2) y(degree)*(resolution/2)];
        straight_end_pose_theta = straight_start_pose_theta;
        straight_end_pose = [end_position straight_end_pose_theta];
        
        end_position_long = [x(degree)*(resolution/2)*long_distance_factor y(degree)*(resolution/2)*long_distance_factor];
        straight_end_pose_long = [end_position_long straight_end_pose_theta];

        [clotoid_check_straight, p_straight, L ] = clotoide(straight_start_pose, straight_end_pose, numberOfIntermediatePose, min_radius);
        [clotoid_check_straight_long, p_straight_long, L ] = clotoide(straight_start_pose, straight_end_pose_long, numberOfIntermediatePose, min_radius);

        primitives(primID).real_start_pose = straight_start_pose;
        primitives(primID).real_end_pose = straight_end_pose;
        primitives(primID).intermediate_poses = p_straight;
        primitives(primID).additionalActionCostMult = forward_penalty;
        primitives(primID).start_orientation = degree-1;
        straigt_prim_end_pose = [straight_end_pose(1)/resolution straight_end_pose(2)/resolution find(angles==straight_end_pose(3))-1];
        primitives(primID).end_pose = straigt_prim_end_pose;
        primID = primID + 1;
        
        primitives(primID).real_start_pose = straight_start_pose;
        primitives(primID).real_end_pose = straight_end_pose_long;
        primitives(primID).intermediate_poses = p_straight_long;
        primitives(primID).additionalActionCostMult = long_forward_penalty;
        primitives(primID).start_orientation = degree-1;
        prim_end_pose_long = [straight_end_pose_long(1)/resolution straight_end_pose_long(2)/resolution find(angles==straight_end_pose_long(3))-1];
        primitives(primID).end_pose = prim_end_pose_long;
        primID = primID + 1;

        if(print_stuff==1)
            plot(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),'+-r');
            plot(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),'+-g');
            grid on;
            hold on;
            quiver(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),...
                cos(primitives(primID-2).intermediate_poses(:,3)),sin(primitives(primID-2).intermediate_poses(:,3)),0.1,...
                'Color',[0 0 0]);
            quiver(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),...
                cos(primitives(primID-1).intermediate_poses(:,3)),sin(primitives(primID-1).intermediate_poses(:,3)),0.1,...
                'Color',[0 0 0]);
            pause(0.1);
        end
        degree = degree + 1; 
    end
end



%% For loop Quadrants:


for degree_number = 1:16
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    if (degree_number == 1 || degree_number == 2 || degree_number == 3 || degree_number == 4) %% Quadrant 1
        if (degree_number == 1)
        % d1
        d1_start_pose_theta = angles(2);
        start_pose_d1 = [start_position d1_start_pose_theta];

        d1_end_pose_theta = angles(16);

        d1_2_start_pose_theta = angles(16);    
        start_pose_d1_2 = [start_position d1_2_start_pose_theta];

        d1_2_end_pose_theta = angles(2);

        for x_position_d1 = for_loop_search
            d1_final_position = [x_position_d1 0];
            end_pose_d1 = [d1_final_position d1_end_pose_theta];
            [clotoid_check_d1, p_d1, L ] = clotoide(start_pose_d1, end_pose_d1, numberOfIntermediatePose, min_radius);

            x_position_d1_2 = x_position_d1;
            d1_2_final_position = [x_position_d1_2 0];
            end_pose_d1_2 = [d1_2_final_position d1_2_end_pose_theta];
            [clotoid_check_d1_2, p_d1_2, L ] = clotoide(start_pose_d1_2, end_pose_d1_2, numberOfIntermediatePose, min_radius);

            if (clotoid_check_d1 == true && clotoid_check_d1_2 == true)
                primitives(primID).real_start_pose = start_pose_d1;
                primitives(primID).real_end_pose = end_pose_d1;
                primitives(primID).intermediate_poses = p_d1;
                primitives(primID).additionalActionCostMult = forward_penalty;
                primitives(primID).start_orientation = find(angles==start_pose_d1(3))-1;
                prim_end_pose = [end_pose_d1(1)/resolution end_pose_d1(2)/resolution find(angles==end_pose_d1(3))-1];
                primitives(primID).end_pose = prim_end_pose;
                primID = primID + 1;

                primitives(primID).real_start_pose = start_pose_d1_2;
                primitives(primID).real_end_pose = end_pose_d1_2;
                primitives(primID).intermediate_poses = p_d1_2;
                primitives(primID).additionalActionCostMult = forward_penalty;
                primitives(primID).start_orientation = find(angles==start_pose_d1_2(3))-1;
                prim_end_pose_2 = [end_pose_d1_2(1)/resolution end_pose_d1_2(2)/resolution find(angles==end_pose_d1_2(3))-1];
                primitives(primID).end_pose = prim_end_pose_2;
                primID = primID + 1;

                if(print_stuff==1)
                    plot(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),'+-b');
                    plot(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),'+-b');
                    grid on; %axis equal;
                    hold on;
                    quiver(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),...
                        cos(primitives(primID-1).intermediate_poses(:,3)),sin(primitives(primID-1).intermediate_poses(:,3)),0.1,...
                        'Color',[0 0 0]);
                    quiver(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),...
                        cos(primitives(primID-2).intermediate_poses(:,3)),sin(primitives(primID-2).intermediate_poses(:,3)),0.1,...
                        'Color',[0 0 0]);
                    pause(0.1);
                end
            break 
            end   
        end
        end

        % d2 and d3
        if(degree_number == 2)
        d2_start_pose_theta = angles(1);
        start_pose_d2 = [start_position d2_start_pose_theta];

        d2_end_pose_theta = angles(3);

        d3_start_pose_theta = angles(3);
        start_pose_d3 = [start_position d3_start_pose_theta];

        d3_end_pose_theta = angles(1);


        for d2_y_position = for_loop_search
            d2_final_position = [d2_y_position*2 d2_y_position];
            end_pose_d2 = [d2_final_position d2_end_pose_theta];
            [clotoid_check_d2, p_d2, L ] = clotoide(start_pose_d2, end_pose_d2, numberOfIntermediatePose, min_radius);

            d3_y_position = d2_y_position;
            d3_final_position = [d3_y_position*2 d3_y_position];
            end_pose_d3 = [d3_final_position d3_end_pose_theta];
            [clotoid_check_d3, p_d3, L ] = clotoide(start_pose_d3, end_pose_d3, numberOfIntermediatePose, min_radius);

            if (clotoid_check_d2 == true && clotoid_check_d3 == true)
                primitives(primID).real_start_pose = start_pose_d2;
                primitives(primID).real_end_pose = end_pose_d2;
                primitives(primID).intermediate_poses = p_d2;
                primitives(primID).additionalActionCostMult = forward_penalty;
                primitives(primID).start_orientation = find(angles==start_pose_d2(3))-1;
                prim_end_pose = [end_pose_d2(1)/resolution end_pose_d2(2)/resolution find(angles==end_pose_d2(3))-1];
                primitives(primID).end_pose = prim_end_pose;
                primID = primID + 1;

                primitives(primID).real_start_pose = start_pose_d3;
                primitives(primID).real_end_pose = end_pose_d3;
                primitives(primID).intermediate_poses = p_d3;
                primitives(primID).additionalActionCostMult = forward_penalty;
                primitives(primID).start_orientation = find(angles==start_pose_d3(3))-1;
                prim_end_pose = [end_pose_d3(1)/resolution end_pose_d3(2)/resolution find(angles==end_pose_d3(3))-1];
                primitives(primID).end_pose = prim_end_pose;
                primID = primID + 1;

                if(print_stuff==1)
                    plot(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),'+-b');
                    plot(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),'+-b');
                    grid on;
                    hold on;
                    quiver(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),...
                        cos(primitives(primID-1).intermediate_poses(:,3)),sin(primitives(primID-1).intermediate_poses(:,3)),0.1,...
                        'Color',[0 0 0]);
                    quiver(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),...
                        cos(primitives(primID-2).intermediate_poses(:,3)),sin(primitives(primID-2).intermediate_poses(:,3)),0.1,...
                        'Color',[0 0 0]);
                    pause(0.1);
                end
            break
            end 
        end
        end

        % d4 
        if(degree_number == 3)
        d4_start_pose_theta = atan2(1,2);
        start_pose_d4 = [start_position d4_start_pose_theta];

        d4_end_pose_theta = atan2(2,1);

        d4_2_start_pose_theta = angles(4);
        start_pose_d4_2 = [start_position d4_2_start_pose_theta];

        d4_2_end_pose_theta = angles(2);

        for d4_x_position = for_loop_search
            d4_y_position = d4_x_position;
            d4_final_position = [d4_x_position d4_y_position];
            end_pose_d4 = [d4_final_position d4_end_pose_theta];
            [clotoid_check_d4, p_d4, L ] = clotoide(start_pose_d4, end_pose_d4, numberOfIntermediatePose, min_radius);

            d4_2_x_position = d4_x_position;
            d4_2_y_position = d4_2_x_position;
            d4_2_final_position = [d4_2_x_position d4_2_y_position];
            end_pose_d4_2 = [d4_2_final_position d4_2_end_pose_theta];
            [clotoid_check_d4_2, p_d4_2, L ] = clotoide(start_pose_d4_2, end_pose_d4_2, numberOfIntermediatePose, min_radius);

            if (clotoid_check_d4 == true && clotoid_check_d4_2 == true)
                primitives(primID).real_start_pose = start_pose_d4;
                primitives(primID).real_end_pose = end_pose_d4;
                primitives(primID).intermediate_poses = p_d4;
                primitives(primID).additionalActionCostMult = forward_penalty;
                primitives(primID).start_orientation = find(angles==start_pose_d4(3))-1;
                prim_end_pose = [end_pose_d4(1)/resolution end_pose_d4(2)/resolution find(angles==end_pose_d4(3))-1];
                primitives(primID).end_pose = prim_end_pose;
                primID = primID + 1;


                primitives(primID).real_start_pose = start_pose_d4_2;
                primitives(primID).real_end_pose = end_pose_d4_2;
                primitives(primID).intermediate_poses = p_d4_2;
                primitives(primID).additionalActionCostMult = forward_penalty;
                primitives(primID).start_orientation = find(angles==start_pose_d4_2(3))-1;
                prim_end_pose = [end_pose_d4_2(1)/resolution end_pose_d4_2(2)/resolution find(angles==end_pose_d4_2(3))-1];
                primitives(primID).end_pose = prim_end_pose;
                primID = primID + 1;

                if(print_stuff==1)
                    plot(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),'+-b');
                    plot(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),'+-b');
                    grid on;
                    hold on;
                    quiver(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),...
                        cos(primitives(primID-1).intermediate_poses(:,3)),sin(primitives(primID-1).intermediate_poses(:,3)),0.1,...
                        'Color',[0 0 0]);
                    quiver(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),...
                        cos(primitives(primID-2).intermediate_poses(:,3)),sin(primitives(primID-2).intermediate_poses(:,3)),0.1,...
                        'Color',[0 0 0]);
                    pause(0.1);
                end
            break
            end 
        end
        end
        
        %  d5 and d6
        if(degree_number == 4)
        d5_start_pose_theta = angles(3);
        start_pose_d5 = [start_position d5_start_pose_theta];

        d5_end_pose_theta = angles(5);

        d6_start_pose_theta = angles(5);
        start_pose_d6 = [start_position d6_start_pose_theta];

        d6_end_pose_theta = angles(3);


        for d5_x_position = for_loop_search
            d5_final_position = [d5_x_position d5_x_position*2];
            end_pose_d5 = [d5_final_position d5_end_pose_theta];
            [clotoid_check_d5, p_d5, L ] = clotoide(start_pose_d5, end_pose_d5, numberOfIntermediatePose, min_radius);

            d6_x_position = d5_x_position;
            d6_final_position = [d6_x_position d6_x_position*2];
            end_pose_d6 = [d6_final_position d6_end_pose_theta];
            [clotoid_check_d6, p_d6, L ] = clotoide(start_pose_d6, end_pose_d6, numberOfIntermediatePose, min_radius);

            if (clotoid_check_d5 == true && clotoid_check_d6 == true)
                primitives(primID).real_start_pose = start_pose_d5;
                primitives(primID).real_end_pose = end_pose_d5;
                primitives(primID).intermediate_poses = p_d5;
                primitives(primID).additionalActionCostMult = forward_penalty;
                primitives(primID).start_orientation = find(angles==start_pose_d5(3))-1;
                prim_end_pose = [end_pose_d5(1)/resolution end_pose_d5(2)/resolution find(angles==end_pose_d5(3))-1];
                primitives(primID).end_pose = prim_end_pose;
                primID = primID + 1;

                primitives(primID).real_start_pose = start_pose_d6;
                primitives(primID).real_end_pose = end_pose_d6;
                primitives(primID).intermediate_poses = p_d6;
                primitives(primID).additionalActionCostMult = forward_penalty;
                primitives(primID).start_orientation = find(angles==start_pose_d6(3))-1;
                prim_end_pose = [end_pose_d6(1)/resolution end_pose_d6(2)/resolution find(angles==end_pose_d6(3))-1];
                primitives(primID).end_pose = prim_end_pose;
                primID = primID + 1;

                if(print_stuff==1)
                    plot(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),'+-b');
                    plot(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),'+-b');
                    grid on;
                    hold on;
                    quiver(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),...
                        cos(primitives(primID-1).intermediate_poses(:,3)),sin(primitives(primID-1).intermediate_poses(:,3)),0.1,...
                        'Color',[0 0 0]);
                    quiver(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),...
                        cos(primitives(primID-2).intermediate_poses(:,3)),sin(primitives(primID-2).intermediate_poses(:,3)),0.1,...
                        'Color',[0 0 0]);
                    pause(0.1);
                end
            break
            end 
        end
        end
     end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   if (degree_number == 5 || degree_number == 6 || degree_number == 7 || degree_number == 8) %% Quadrant 2
        if (degree_number == 5)
        % d1
        d1_start_pose_theta = angles(4);
        start_pose_d1 = [start_position d1_start_pose_theta];

        d1_end_pose_theta = angles(6);

        d1_2_start_pose_theta = angles(6);    
        start_pose_d1_2 = [start_position d1_2_start_pose_theta];

        d1_2_end_pose_theta = angles(4);

        for y_position_d1 = for_loop_search
            d1_final_position = [0 y_position_d1];
            end_pose_d1 = [d1_final_position d1_end_pose_theta];
            [clotoid_check_d1, p_d1, L ] = clotoide(start_pose_d1, end_pose_d1, numberOfIntermediatePose, min_radius);

            y_position_d1_2 = y_position_d1;
            d1_2_final_position = [0 y_position_d1_2];
            end_pose_d1_2 = [d1_2_final_position d1_2_end_pose_theta];
            [clotoid_check_d1_2, p_d1_2, L ] = clotoide(start_pose_d1_2, end_pose_d1_2, numberOfIntermediatePose, min_radius);

            if (clotoid_check_d1 == true && clotoid_check_d1_2 == true)
                primitives(primID).real_start_pose = start_pose_d1;
                primitives(primID).real_end_pose = end_pose_d1;
                primitives(primID).intermediate_poses = p_d1;
                primitives(primID).additionalActionCostMult = forward_penalty;
                
                primitives(primID).start_orientation = find(angles==start_pose_d1(3))-1;
                prim_end_pose = [end_pose_d1(1)/resolution end_pose_d1(2)/resolution find(angles==end_pose_d1(3))-1];
                primitives(primID).end_pose = prim_end_pose;
                primID = primID + 1;

                primitives(primID).real_start_pose = start_pose_d1_2;
                primitives(primID).real_end_pose = end_pose_d1_2;
                primitives(primID).intermediate_poses = p_d1_2;
                primitives(primID).additionalActionCostMult = forward_penalty;
                
                primitives(primID).start_orientation = find(angles==start_pose_d1_2(3))-1;
                prim_end_pose = [end_pose_d1_2(1)/resolution end_pose_d1_2(2)/resolution find(angles==end_pose_d1_2(3))-1];
                primitives(primID).end_pose = prim_end_pose;
                primID = primID + 1;

                if(print_stuff==1)
                    plot(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),'+-b');
                    plot(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),'+-b');
                    grid on; %axis equal;
                    hold on;
                    quiver(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),...
                        cos(primitives(primID-1).intermediate_poses(:,3)),sin(primitives(primID-1).intermediate_poses(:,3)),0.1,...
                        'Color',[0 0 0]);
                    quiver(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),...
                        cos(primitives(primID-2).intermediate_poses(:,3)),sin(primitives(primID-2).intermediate_poses(:,3)),0.1,...
                        'Color',[0 0 0]);
                    pause(0.1);
                end
            break 
            end   
        end
        end

        % d2 and d3
        if(degree_number == 6)
        d2_start_pose_theta = angles(5);
        start_pose_d2 = [start_position d2_start_pose_theta];

        d2_end_pose_theta = angles(7);

        d3_start_pose_theta = angles(7);
        start_pose_d3 = [start_position d3_start_pose_theta];

        d3_end_pose_theta = angles(5);


        for d2_x_position = for_loop_search
            d2_final_position = [-d2_x_position d2_x_position*2];
            end_pose_d2 = [d2_final_position d2_end_pose_theta];
            [clotoid_check_d2, p_d2, L ] = clotoide(start_pose_d2, end_pose_d2, numberOfIntermediatePose, min_radius);

            d3_x_position = d2_x_position;
            d3_final_position = [-d3_x_position d3_x_position*2];
            end_pose_d3 = [d3_final_position d3_end_pose_theta];
            [clotoid_check_d3, p_d3, L ] = clotoide(start_pose_d3, end_pose_d3, numberOfIntermediatePose, min_radius);

            if (clotoid_check_d2 == true && clotoid_check_d3 == true)
                primitives(primID).real_start_pose = start_pose_d2;
                primitives(primID).real_end_pose = end_pose_d2;
                primitives(primID).intermediate_poses = p_d2;
                primitives(primID).additionalActionCostMult = forward_penalty;

                primitives(primID).start_orientation = find(angles==start_pose_d2(3))-1;
                prim_end_pose = [end_pose_d2(1)/resolution end_pose_d2(2)/resolution find(angles==end_pose_d2(3))-1];
                primitives(primID).end_pose = prim_end_pose;
                primID = primID + 1;

                primitives(primID).real_start_pose = start_pose_d3;
                primitives(primID).real_end_pose = end_pose_d3;
                primitives(primID).intermediate_poses = p_d3;
                primitives(primID).additionalActionCostMult = forward_penalty;

                primitives(primID).start_orientation = find(angles==start_pose_d3(3))-1;
                prim_end_pose = [end_pose_d3(1)/resolution end_pose_d3(2)/resolution find(angles==end_pose_d3(3))-1];
                primitives(primID).end_pose = prim_end_pose;
                primID = primID + 1;

                if(print_stuff==1)
                    plot(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),'+-b');
                    plot(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),'+-b');
                    grid on;
                    hold on;
                    quiver(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),...
                        cos(primitives(primID-1).intermediate_poses(:,3)),sin(primitives(primID-1).intermediate_poses(:,3)),0.1,...
                        'Color',[0 0 0]);
                    quiver(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),...
                        cos(primitives(primID-2).intermediate_poses(:,3)),sin(primitives(primID-2).intermediate_poses(:,3)),0.1,...
                        'Color',[0 0 0]);
                    pause(0.1);
                end
            break
            end 
        end
        end

        % d4 
        if(degree_number == 7)
        d4_start_pose_theta = angles(6);
        start_pose_d4 = [start_position d4_start_pose_theta];

        d4_end_pose_theta = angles(8);

        d4_2_start_pose_theta = angles(8);
        start_pose_d4_2 = [start_position d4_2_start_pose_theta];

        d4_2_end_pose_theta = angles(6);

        for d4_x_position = for_loop_search
            d4_y_position = d4_x_position;
            d4_final_position = [-d4_x_position d4_y_position];
            end_pose_d4 = [d4_final_position d4_end_pose_theta];
            [clotoid_check_d4, p_d4, L ] = clotoide(start_pose_d4, end_pose_d4, numberOfIntermediatePose, min_radius);

            d4_2_x_position = d4_x_position;
            d4_2_y_position = d4_2_x_position;
            d4_2_final_position = [-d4_2_x_position d4_2_y_position];
            end_pose_d4_2 = [d4_2_final_position d4_2_end_pose_theta];
            [clotoid_check_d4_2, p_d4_2, L ] = clotoide(start_pose_d4_2, end_pose_d4_2, numberOfIntermediatePose, min_radius);

            if (clotoid_check_d4 == true && clotoid_check_d4_2 == true)
                primitives(primID).real_start_pose = start_pose_d4;
                primitives(primID).real_end_pose = end_pose_d4;
                primitives(primID).intermediate_poses = p_d4;
                primitives(primID).additionalActionCostMult = forward_penalty;

                primitives(primID).start_orientation = find(angles==start_pose_d4(3))-1;
                prim_end_pose = [end_pose_d4(1)/resolution end_pose_d4(2)/resolution find(angles==end_pose_d4(3))-1];
                primitives(primID).end_pose = prim_end_pose;
                primID = primID + 1;


                primitives(primID).real_start_pose = start_pose_d4_2;
                primitives(primID).real_end_pose = end_pose_d4_2;
                primitives(primID).intermediate_poses = p_d4_2;
                primitives(primID).additionalActionCostMult = forward_penalty;

                primitives(primID).start_orientation = find(angles==start_pose_d4_2(3))-1;
                prim_end_pose = [end_pose_d4_2(1)/resolution end_pose_d4_2(2)/resolution find(angles==end_pose_d4_2(3))-1];
                primitives(primID).end_pose = prim_end_pose;
                primID = primID + 1;

                if(print_stuff==1)
                    plot(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),'+-b');
                    plot(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),'+-b');
                    grid on;
                    hold on;
                    quiver(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),...
                        cos(primitives(primID-1).intermediate_poses(:,3)),sin(primitives(primID-1).intermediate_poses(:,3)),0.1,...
                        'Color',[0 0 0]);
                    quiver(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),...
                        cos(primitives(primID-2).intermediate_poses(:,3)),sin(primitives(primID-2).intermediate_poses(:,3)),0.1,...
                        'Color',[0 0 0]);
                    pause(0.1);
                end
            break
            end 
        end
        end
        
        % d5 and d6
        if(degree_number == 8)
        d5_start_pose_theta = angles(7);
        start_pose_d5 = [start_position d5_start_pose_theta];

        d5_end_pose_theta = angles(9);

        d6_start_pose_theta = angles(9);
        start_pose_d6 = [start_position d6_start_pose_theta];

        d6_end_pose_theta = angles(7);


        for d5_y_position = for_loop_search
            d5_final_position = [-d5_y_position*2 d5_y_position];
            end_pose_d5 = [d5_final_position d5_end_pose_theta];
            [clotoid_check_d5, p_d5, L ] = clotoide(start_pose_d5, end_pose_d5, numberOfIntermediatePose, min_radius);

            d6_y_position = d5_y_position;
            d6_final_position = [-d6_y_position*2 d6_y_position];
            end_pose_d6 = [d6_final_position d6_end_pose_theta];
            [clotoid_check_d6, p_d6, L ] = clotoide(start_pose_d6, end_pose_d6, numberOfIntermediatePose, min_radius);

            if (clotoid_check_d5 == true && clotoid_check_d6 == true)
                primitives(primID).real_start_pose = start_pose_d5;
                primitives(primID).real_end_pose = end_pose_d5;
                primitives(primID).intermediate_poses = p_d5;
                primitives(primID).additionalActionCostMult = forward_penalty;

                primitives(primID).start_orientation = find(angles==start_pose_d5(3))-1;
                prim_end_pose = [end_pose_d5(1)/resolution end_pose_d5(2)/resolution find(angles==end_pose_d5(3))-1];
                primitives(primID).end_pose = prim_end_pose;
                primID = primID + 1;

                primitives(primID).real_start_pose = start_pose_d6;
                primitives(primID).real_end_pose = end_pose_d6;
                primitives(primID).intermediate_poses = p_d6;
                primitives(primID).additionalActionCostMult = forward_penalty;
                primitives(primID).start_orientation = find(angles==start_pose_d6(3))-1;
                prim_end_pose = [end_pose_d6(1)/resolution end_pose_d6(2)/resolution find(angles==end_pose_d6(3))-1];
                primitives(primID).end_pose = prim_end_pose;
                primID = primID + 1;
                
                if(print_stuff==1)
                    plot(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),'+-b');
                    plot(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),'+-b');
                    grid on;
                    hold on;
                    quiver(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),...
                        cos(primitives(primID-1).intermediate_poses(:,3)),sin(primitives(primID-1).intermediate_poses(:,3)),0.1,...
                        'Color',[0 0 0]);
                    quiver(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),...
                        cos(primitives(primID-2).intermediate_poses(:,3)),sin(primitives(primID-2).intermediate_poses(:,3)),0.1,...
                        'Color',[0 0 0]);
                    pause(0.1);
                end
            break
            end 
        end
        end    
    end
    
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    if (degree_number == 9 || degree_number == 10 || degree_number == 11 || degree_number == 12) %% Quadrant 3
        
    if (degree_number == 9)
        % d1
        d1_start_pose_theta = angles(8);
        start_pose_d1 = [start_position d1_start_pose_theta];

        d1_end_pose_theta = angles(10);

        d1_2_start_pose_theta = angles(10);    
        start_pose_d1_2 = [start_position d1_2_start_pose_theta];

        d1_2_end_pose_theta = angles(8);

        for x_position_d1 = for_loop_search
            d1_final_position = [-x_position_d1 0];
            end_pose_d1 = [d1_final_position d1_end_pose_theta];
            [clotoid_check_d1, p_d1, L ] = clotoide(start_pose_d1, end_pose_d1, numberOfIntermediatePose, min_radius);

            x_position_d1_2 = x_position_d1;
            d1_2_final_position = [-x_position_d1_2 0];
            end_pose_d1_2 = [d1_2_final_position d1_2_end_pose_theta];
            [clotoid_check_d1_2, p_d1_2, L ] = clotoide(start_pose_d1_2, end_pose_d1_2, numberOfIntermediatePose, min_radius);

            if (clotoid_check_d1 == true && clotoid_check_d1_2 == true)
                primitives(primID).real_start_pose = start_pose_d1;
                primitives(primID).real_end_pose = end_pose_d1;
                primitives(primID).intermediate_poses = p_d1;
                primitives(primID).additionalActionCostMult = forward_penalty;
                
                primitives(primID).start_orientation = find(angles==start_pose_d1(3))-1;
                prim_end_pose = [end_pose_d1(1)/resolution end_pose_d1(2)/resolution find(angles==end_pose_d1(3))-1];
                primitives(primID).end_pose = prim_end_pose;
                primID = primID + 1;

                primitives(primID).real_start_pose = start_pose_d1_2;
                primitives(primID).real_end_pose = end_pose_d1_2;
                primitives(primID).intermediate_poses = p_d1_2;
                primitives(primID).additionalActionCostMult = forward_penalty;
                
                primitives(primID).start_orientation = find(angles==start_pose_d1_2(3))-1;
                prim_end_pose = [end_pose_d1_2(1)/resolution end_pose_d1_2(2)/resolution find(angles==end_pose_d1_2(3))-1];
                primitives(primID).end_pose = prim_end_pose;
                primID = primID + 1;

                if(print_stuff==1)
                    plot(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),'+-b');
                    plot(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),'+-b');
                    grid on; %axis equal;
                    hold on;
                    quiver(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),...
                        cos(primitives(primID-1).intermediate_poses(:,3)),sin(primitives(primID-1).intermediate_poses(:,3)),0.1,...
                        'Color',[0 0 0]);
                    quiver(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),...
                        cos(primitives(primID-2).intermediate_poses(:,3)),sin(primitives(primID-2).intermediate_poses(:,3)),0.1,...
                        'Color',[0 0 0]);
                    pause(0.1);
                end
            break 
            end   
        end
    end

        
        % d2 and d3
        if(degree_number == 10)
        d2_start_pose_theta = angles(9);
        start_pose_d2 = [start_position d2_start_pose_theta];

        d2_end_pose_theta = angles(11);

        d3_start_pose_theta = angles(11);
        start_pose_d3 = [start_position d3_start_pose_theta];

        d3_end_pose_theta = angles(9);


        for d2_y_position = for_loop_search
            d2_final_position = [-d2_y_position*2 -d2_y_position];
            end_pose_d2 = [d2_final_position d2_end_pose_theta];
            [clotoid_check_d2, p_d2, L ] = clotoide(start_pose_d2, end_pose_d2, numberOfIntermediatePose, min_radius);

            d3_y_position = d2_y_position;
            d3_final_position = [-d3_y_position*2 -d3_y_position];
            end_pose_d3 = [d3_final_position d3_end_pose_theta];
            [clotoid_check_d3, p_d3, L ] = clotoide(start_pose_d3, end_pose_d3, numberOfIntermediatePose, min_radius);

            if (clotoid_check_d2 == true && clotoid_check_d3 == true)
                primitives(primID).real_start_pose = start_pose_d2;
                primitives(primID).real_end_pose = end_pose_d2;
                primitives(primID).intermediate_poses = p_d2;
                primitives(primID).additionalActionCostMult = forward_penalty;

                primitives(primID).start_orientation = find(angles==start_pose_d2(3))-1;
                prim_end_pose = [end_pose_d2(1)/resolution end_pose_d2(2)/resolution find(angles==end_pose_d2(3))-1];
                primitives(primID).end_pose = prim_end_pose;
                primID = primID + 1;

                primitives(primID).real_start_pose = start_pose_d3;
                primitives(primID).real_end_pose = end_pose_d3;
                primitives(primID).intermediate_poses = p_d3;
                primitives(primID).additionalActionCostMult = forward_penalty;

                primitives(primID).start_orientation = find(angles==start_pose_d3(3))-1;
                prim_end_pose = [end_pose_d3(1)/resolution end_pose_d3(2)/resolution find(angles==end_pose_d3(3))-1];
                primitives(primID).end_pose = prim_end_pose;
                primID = primID + 1;

                if(print_stuff==1)
                    plot(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),'+-b');
                    plot(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),'+-b');
                    grid on;
                    hold on;
                    quiver(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),...
                        cos(primitives(primID-1).intermediate_poses(:,3)),sin(primitives(primID-1).intermediate_poses(:,3)),0.1,...
                        'Color',[0 0 0]);
                    quiver(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),...
                        cos(primitives(primID-2).intermediate_poses(:,3)),sin(primitives(primID-2).intermediate_poses(:,3)),0.1,...
                        'Color',[0 0 0]);
                    pause(0.1);
                end
            break
            end 
        end
        end

        % d4 
        if(degree_number == 11)
        d4_start_pose_theta = angles(10);
        start_pose_d4 = [start_position d4_start_pose_theta];

        d4_end_pose_theta = angles(12);

        d4_2_start_pose_theta = angles(12);
        start_pose_d4_2 = [start_position d4_2_start_pose_theta];

        d4_2_end_pose_theta = angles(10);

        for d4_x_position = for_loop_search
            d4_y_position = d4_x_position;
            d4_final_position = [-d4_x_position -d4_y_position];
            end_pose_d4 = [d4_final_position d4_end_pose_theta];
            [clotoid_check_d4, p_d4, L ] = clotoide(start_pose_d4, end_pose_d4, numberOfIntermediatePose, min_radius);

            d4_2_x_position = d4_x_position;
            d4_2_y_position = d4_2_x_position;
            d4_2_final_position = [-d4_2_x_position -d4_2_y_position];
            end_pose_d4_2 = [d4_2_final_position d4_2_end_pose_theta];
            [clotoid_check_d4_2, p_d4_2, L ] = clotoide(start_pose_d4_2, end_pose_d4_2, numberOfIntermediatePose, min_radius);

            if (clotoid_check_d4 == true && clotoid_check_d4_2 == true)
                primitives(primID).real_start_pose = start_pose_d4;
                primitives(primID).real_end_pose = end_pose_d4;
                primitives(primID).intermediate_poses = p_d4;
                primitives(primID).additionalActionCostMult = forward_penalty;
                
                primitives(primID).start_orientation = find(angles==start_pose_d4(3))-1;
                prim_end_pose = [end_pose_d4(1)/resolution end_pose_d4(2)/resolution find(angles==end_pose_d4(3))-1];
                primitives(primID).end_pose = prim_end_pose;
                primID = primID + 1;


                primitives(primID).real_start_pose = start_pose_d4_2;
                primitives(primID).real_end_pose = end_pose_d4_2;
                primitives(primID).intermediate_poses = p_d4_2;
                primitives(primID).additionalActionCostMult = forward_penalty;

                primitives(primID).start_orientation = find(angles==start_pose_d4_2(3))-1;
                prim_end_pose = [end_pose_d4_2(1)/resolution end_pose_d4_2(2)/resolution find(angles==end_pose_d4_2(3))-1];
                primitives(primID).end_pose = prim_end_pose;
                primID = primID + 1;

                if(print_stuff==1)
                    plot(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),'+-b');
                    plot(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),'+-b');
                    grid on;
                    hold on;
                    quiver(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),...
                        cos(primitives(primID-1).intermediate_poses(:,3)),sin(primitives(primID-1).intermediate_poses(:,3)),0.1,...
                        'Color',[0 0 0]);
                    quiver(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),...
                        cos(primitives(primID-2).intermediate_poses(:,3)),sin(primitives(primID-2).intermediate_poses(:,3)),0.1,...
                        'Color',[0 0 0]);
                    pause(0.1);
                end
            break
            end 
        end
        end
        
        % d5 and d6
        if(degree_number == 12)
        d5_start_pose_theta = angles(11);
        start_pose_d5 = [start_position d5_start_pose_theta];

        d5_end_pose_theta = angles(13);

        d6_start_pose_theta = angles(13);
        start_pose_d6 = [start_position d6_start_pose_theta];

        d6_end_pose_theta = angles(11);


        for d5_x_position = for_loop_search
            d5_final_position = [-d5_x_position -d5_x_position*2];
            end_pose_d5 = [d5_final_position d5_end_pose_theta];
            [clotoid_check_d5, p_d5, L ] = clotoide(start_pose_d5, end_pose_d5, numberOfIntermediatePose, min_radius);

            d6_x_position = d5_x_position;
            d6_final_position = [-d6_x_position -d6_x_position*2];
            end_pose_d6 = [d6_final_position d6_end_pose_theta];
            [clotoid_check_d6, p_d6, L ] = clotoide(start_pose_d6, end_pose_d6, numberOfIntermediatePose, min_radius);

            if (clotoid_check_d5 == true && clotoid_check_d6 == true)
                primitives(primID).real_start_pose = start_pose_d5;
                primitives(primID).real_end_pose = end_pose_d5;
                primitives(primID).intermediate_poses = p_d5;
                primitives(primID).additionalActionCostMult = forward_penalty;
                  
                primitives(primID).start_orientation = find(angles==start_pose_d5(3))-1;
                prim_end_pose = [end_pose_d5(1)/resolution end_pose_d5(2)/resolution find(angles==end_pose_d5(3))-1];
                primitives(primID).end_pose = prim_end_pose;
                primID = primID + 1;

                primitives(primID).real_start_pose = start_pose_d6;
                primitives(primID).real_end_pose = end_pose_d6;
                primitives(primID).intermediate_poses = p_d6;
                primitives(primID).additionalActionCostMult = forward_penalty;
                primitives(primID).start_orientation = find(angles==start_pose_d6(3))-1;
                prim_end_pose = [end_pose_d6(1)/resolution end_pose_d6(2)/resolution find(angles==end_pose_d6(3))-1];
                primitives(primID).end_pose = prim_end_pose;
                primID = primID + 1;

                if(print_stuff==1)
                    plot(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),'+-b');
                    plot(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),'+-b');
                    grid on;
                    hold on;
                    quiver(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),...
                        cos(primitives(primID-1).intermediate_poses(:,3)),sin(primitives(primID-1).intermediate_poses(:,3)),0.1,...
                        'Color',[0 0 0]);
                    quiver(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),...
                        cos(primitives(primID-2).intermediate_poses(:,3)),sin(primitives(primID-2).intermediate_poses(:,3)),0.1,...
                        'Color',[0 0 0]);
                    pause(0.1);
                end
            break
            end 
        end
        end      
     end
    
    
    
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    if (degree_number == 13 || degree_number == 14 || degree_number == 15 || degree_number == 16) %% Quadrant 4
        if (degree_number == 13)
        % d1
        d1_start_pose_theta = angles(12);
        start_pose_d1 = [start_position d1_start_pose_theta];

        d1_end_pose_theta = angles(14);

        d1_2_start_pose_theta = angles(14);    
        start_pose_d1_2 = [start_position d1_2_start_pose_theta];

        d1_2_end_pose_theta = angles(12);

        for y_position_d1 = for_loop_search
            d1_final_position = [0 -y_position_d1];
            end_pose_d1 = [d1_final_position d1_end_pose_theta];
            [clotoid_check_d1, p_d1, L ] = clotoide(start_pose_d1, end_pose_d1, numberOfIntermediatePose, min_radius);

            y_position_d1_2 = y_position_d1;
            d1_2_final_position = [0 -y_position_d1_2];
            end_pose_d1_2 = [d1_2_final_position d1_2_end_pose_theta];
            [clotoid_check_d1_2, p_d1_2, L ] = clotoide(start_pose_d1_2, end_pose_d1_2, numberOfIntermediatePose, min_radius);

            if (clotoid_check_d1 == true && clotoid_check_d1_2 == true)
                primitives(primID).real_start_pose = start_pose_d1;
                primitives(primID).real_end_pose = end_pose_d1;
                primitives(primID).intermediate_poses = p_d1;
                primitives(primID).additionalActionCostMult = forward_penalty;
                
                primitives(primID).start_orientation = find(angles==start_pose_d1(3))-1;
                prim_end_pose = [end_pose_d1(1)/resolution end_pose_d1(2)/resolution find(angles==end_pose_d1(3))-1];
                primitives(primID).end_pose = prim_end_pose;
                primID = primID + 1;

                primitives(primID).real_start_pose = start_pose_d1_2;
                primitives(primID).real_end_pose = end_pose_d1_2;
                primitives(primID).intermediate_poses = p_d1_2;
                primitives(primID).additionalActionCostMult = forward_penalty;
                
                primitives(primID).start_orientation = find(angles==start_pose_d1_2(3))-1;
                prim_end_pose = [end_pose_d1_2(1)/resolution end_pose_d1_2(2)/resolution find(angles==end_pose_d1_2(3))-1];
                primitives(primID).end_pose = prim_end_pose;
                primID = primID + 1;

                if(print_stuff==1)
                    plot(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),'+-b');
                    plot(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),'+-b');
                    grid on; %axis equal;
                    hold on;
                    quiver(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),...
                        cos(primitives(primID-1).intermediate_poses(:,3)),sin(primitives(primID-1).intermediate_poses(:,3)),0.1,...
                        'Color',[0 0 0]);
                    quiver(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),...
                        cos(primitives(primID-2).intermediate_poses(:,3)),sin(primitives(primID-2).intermediate_poses(:,3)),0.1,...
                        'Color',[0 0 0]);
                    pause(0.1);
                end
            break 
            end   
        end
        end

        % d2 and d3
        if(degree_number == 14)
        d2_start_pose_theta = angles(13);
        start_pose_d2 = [start_position d2_start_pose_theta];

        d2_end_pose_theta = angles(15);

        d3_start_pose_theta = angles(15);
        start_pose_d3 = [start_position d3_start_pose_theta];

        d3_end_pose_theta = angles(13);


        for d2_x_position = for_loop_search
            d2_final_position = [d2_x_position -d2_x_position*2];
            end_pose_d2 = [d2_final_position d2_end_pose_theta];
            [clotoid_check_d2, p_d2, L ] = clotoide(start_pose_d2, end_pose_d2, numberOfIntermediatePose, min_radius);

            d3_x_position = d2_x_position;
            d3_final_position = [d3_x_position -d3_x_position*2];
            end_pose_d3 = [d3_final_position d3_end_pose_theta];
            [clotoid_check_d3, p_d3, L ] = clotoide(start_pose_d3, end_pose_d3, numberOfIntermediatePose, min_radius);

            if (clotoid_check_d2 == true && clotoid_check_d3 == true)
                primitives(primID).real_start_pose = start_pose_d2;
                primitives(primID).real_end_pose = end_pose_d2;
                primitives(primID).intermediate_poses = p_d2;
                primitives(primID).additionalActionCostMult = forward_penalty;
                
                primitives(primID).start_orientation = find(angles==start_pose_d2(3))-1;
                prim_end_pose = [end_pose_d2(1)/resolution end_pose_d2(2)/resolution find(angles==end_pose_d2(3))-1];
                primitives(primID).end_pose = prim_end_pose;
                primID = primID + 1;

                primitives(primID).real_start_pose = start_pose_d3;
                primitives(primID).real_end_pose = end_pose_d3;
                primitives(primID).intermediate_poses = p_d3;
                primitives(primID).additionalActionCostMult = forward_penalty;

                primitives(primID).start_orientation = find(angles==start_pose_d3(3))-1;
                prim_end_pose = [end_pose_d3(1)/resolution end_pose_d3(2)/resolution find(angles==end_pose_d3(3))-1];
                primitives(primID).end_pose = prim_end_pose;
                primID = primID + 1;

                if(print_stuff==1)
                    plot(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),'+-b');
                    plot(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),'+-b');
                    grid on;
                    hold on;
                    quiver(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),...
                        cos(primitives(primID-1).intermediate_poses(:,3)),sin(primitives(primID-1).intermediate_poses(:,3)),0.1,...
                        'Color',[0 0 0]);
                    quiver(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),...
                        cos(primitives(primID-2).intermediate_poses(:,3)),sin(primitives(primID-2).intermediate_poses(:,3)),0.1,...
                        'Color',[0 0 0]);
                    pause(0.1);
                end
            break
            end 
        end
        end

        % d4 
        if(degree_number == 15)
        d4_start_pose_theta = angles(14);
        start_pose_d4 = [start_position d4_start_pose_theta];

        d4_end_pose_theta = angles(16);

        d4_2_start_pose_theta = angles(16);
        start_pose_d4_2 = [start_position d4_2_start_pose_theta];

        d4_2_end_pose_theta = angles(14);

        for d4_x_position = for_loop_search
            d4_y_position = d4_x_position;
            d4_final_position = [d4_x_position -d4_y_position];
            end_pose_d4 = [d4_final_position d4_end_pose_theta];
            [clotoid_check_d4, p_d4, L ] = clotoide(start_pose_d4, end_pose_d4, numberOfIntermediatePose, min_radius);

            d4_2_x_position = d4_x_position;
            d4_2_y_position = d4_2_x_position;
            d4_2_final_position = [d4_2_x_position -d4_2_y_position];
            end_pose_d4_2 = [d4_2_final_position d4_2_end_pose_theta];
            [clotoid_check_d4_2, p_d4_2, L ] = clotoide(start_pose_d4_2, end_pose_d4_2, numberOfIntermediatePose, min_radius);

            if (clotoid_check_d4 == true && clotoid_check_d4_2 == true)
                primitives(primID).real_start_pose = start_pose_d4;
                primitives(primID).real_end_pose = end_pose_d4;
                primitives(primID).intermediate_poses = p_d4;
                primitives(primID).additionalActionCostMult = forward_penalty;
                
                primitives(primID).start_orientation = find(angles==start_pose_d4(3))-1;                
                prim_end_pose = [end_pose_d4(1)/resolution end_pose_d4(2)/resolution find(angles==end_pose_d4(3))-1];
                primitives(primID).end_pose = prim_end_pose;
                primID = primID + 1;

                primitives(primID).real_start_pose = start_pose_d4_2;
                primitives(primID).real_end_pose = end_pose_d4_2;
                primitives(primID).intermediate_poses = p_d4_2;
                primitives(primID).additionalActionCostMult = forward_penalty;

                primitives(primID).start_orientation = find(angles==start_pose_d4_2(3))-1;
                prim_end_pose = [end_pose_d4_2(1)/resolution end_pose_d4_2(2)/resolution find(angles==end_pose_d4_2(3))-1];
                primitives(primID).end_pose = prim_end_pose;
                primID = primID + 1;

                if(print_stuff==1)
                    plot(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),'+-b');
                    plot(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),'+-b');
                    grid on;
                    hold on;
                    quiver(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),...
                        cos(primitives(primID-1).intermediate_poses(:,3)),sin(primitives(primID-1).intermediate_poses(:,3)),0.1,...
                        'Color',[0 0 0]);
                    quiver(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),...
                        cos(primitives(primID-2).intermediate_poses(:,3)),sin(primitives(primID-2).intermediate_poses(:,3)),0.1,...
                        'Color',[0 0 0]);
                    pause(0.1);
                end
            break
            end 
        end
        end
        
        % d5 and d6
        if(degree_number == 16)
        d5_start_pose_theta = angles(15);
        start_pose_d5 = [start_position d5_start_pose_theta];

        d5_end_pose_theta = angles(1);

        d6_start_pose_theta = angles(1);
        start_pose_d6 = [start_position d6_start_pose_theta];

        d6_end_pose_theta = angles(15);


        for d5_y_position = for_loop_search
            d5_final_position = [d5_y_position*2 -d5_y_position];
            end_pose_d5 = [d5_final_position d5_end_pose_theta];
            [clotoid_check_d5, p_d5, L ] = clotoide(start_pose_d5, end_pose_d5, numberOfIntermediatePose, min_radius);

            d6_y_position = d5_y_position;
            d6_final_position = [d6_y_position*2 -d6_y_position];
            end_pose_d6 = [d6_final_position d6_end_pose_theta];
            [clotoid_check_d6, p_d6, L ] = clotoide(start_pose_d6, end_pose_d6, numberOfIntermediatePose, min_radius);

            if (clotoid_check_d5 == true && clotoid_check_d6 == true)
                primitives(primID).real_start_pose = start_pose_d5;
                primitives(primID).real_end_pose = end_pose_d5;
                primitives(primID).intermediate_poses = p_d5;
                primitives(primID).additionalActionCostMult = forward_penalty;

                primitives(primID).start_orientation = find(angles==start_pose_d5(3))-1;
                prim_end_pose = [end_pose_d5(1)/resolution end_pose_d5(2)/resolution find(angles==end_pose_d5(3))-1];
                primitives(primID).end_pose = prim_end_pose;
                primID = primID + 1;

                primitives(primID).real_start_pose = start_pose_d6;
                primitives(primID).real_end_pose = end_pose_d6;
                primitives(primID).intermediate_poses = p_d6;
                primitives(primID).additionalActionCostMult = forward_penalty;
                primitives(primID).start_orientation = find(angles==start_pose_d6(3))-1;
                prim_end_pose = [end_pose_d6(1)/resolution end_pose_d6(2)/resolution find(angles==end_pose_d6(3))-1];
                primitives(primID).end_pose = prim_end_pose;
                primID = primID + 1;

                if(print_stuff==1)
                    plot(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),'+-b');
                    plot(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),'+-b');
                    grid on;
                    hold on;
                    quiver(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),...
                        cos(primitives(primID-1).intermediate_poses(:,3)),sin(primitives(primID-1).intermediate_poses(:,3)),0.1,...
                        'Color',[0 0 0]);
                    quiver(primitives(primID-2).intermediate_poses(:,1),primitives(primID-2).intermediate_poses(:,2),...
                        cos(primitives(primID-2).intermediate_poses(:,3)),sin(primitives(primID-2).intermediate_poses(:,3)),0.1,...
                        'Color',[0 0 0]);
                    pause(0.1);
                end
            break
            end 
        end
        end    
    end
   
    
end



set(gca,'DataAspectRatio',[1 1 1]);

generate_file(primitives, numberOfAngle, resolution);

