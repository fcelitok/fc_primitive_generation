%% In this program we are calculating primitive distances according to given Ackermann vehicles
%
%function is working with max_steering_angle, car wheelbase and resolution


function [distance] = primitive_distance_generator(steering_angle,wheelbase,resolution)

max_dist = 5.0; %tested up tp 2.0m
numberOfIntermediatePose = 10;

steering_angle_max = steering_angle*pi/180; % in radiants
min_radius = wheelbase/tan(steering_angle_max);

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

%% d1

d1_start_pose_theta = atan2(1,2);
start_pose_d1 = [start_position d1_start_pose_theta];

d1_end_pose_theta = atan2(-1,2);


for x_position_d1 = for_loop_search
    d1_final_position = [x_position_d1 0];
    end_pose_d1 = [d1_final_position d1_end_pose_theta];
    [clotoid_check_d1, p_d1, L ] = clotoide(start_pose_d1, end_pose_d1, numberOfIntermediatePose, min_radius);
    
    if (clotoid_check_d1 == true)
        primitives(primID).real_start_pose = start_pose_d1;
        primitives(primID).real_end_pose = end_pose_d1;
        primitives(primID).intermediate_poses = p_d1;
        primID = primID + 1;

        if(print_stuff==1)
            plot(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),'+-b');
            grid on; %axis equal;
            hold on;
            quiver(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),...
                cos(primitives(primID-1).intermediate_poses(:,3)),sin(primitives(primID-1).intermediate_poses(:,3)),0.1,...
                'Color',[0 0 0]);
            pause(0.1);
        end
    break 
    end   
end


%% d2 and d3

d2_start_pose_theta = atan2(2,2);
start_pose_d2 = [start_position d2_start_pose_theta];

d2_end_pose_theta = atan2(0,2);

d3_start_pose_theta = atan2(0,2);
start_pose_d3 = [start_position d3_start_pose_theta];

d3_end_pose_theta = atan2(2,2);


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
        primID = primID + 1;

        primitives(primID).real_start_pose = start_pose_d3;
        primitives(primID).real_end_pose = end_pose_d3;
        primitives(primID).intermediate_poses = p_d3;
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



%% d4 

d4_start_pose_theta = atan2(1,2);
start_pose_d4 = [start_position d4_start_pose_theta];

d4_end_pose_theta = atan2(2,1);

for d4_x_position = for_loop_search
    d4_y_position = d4_x_position;
    d4_final_position = [d4_x_position d4_y_position];
    end_pose_d4 = [d4_final_position d4_end_pose_theta];
    [clotoid_check_d4, p_d4, L ] = clotoide(start_pose_d4, end_pose_d4, numberOfIntermediatePose, min_radius);
    
    if (clotoid_check_d4 == true )
        primitives(primID).real_start_pose = start_pose_d4;
        primitives(primID).real_end_pose = end_pose_d4;
        primitives(primID).intermediate_poses = p_d4;
        primID = primID + 1;

        if(print_stuff==1)
            plot(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),'+-b');
            grid on;
            hold on;
            quiver(primitives(primID-1).intermediate_poses(:,1),primitives(primID-1).intermediate_poses(:,2),...
                cos(primitives(primID-1).intermediate_poses(:,3)),sin(primitives(primID-1).intermediate_poses(:,3)),0.1,...
                'Color',[0 0 0]);
            pause(0.1);
        end
    break
    end 
end

%set(gca,'DataAspectRatio',[1 1 1]);
axis equal;
%%
d1_final_position
d1_distance = sqrt(d1_final_position(1)^2+d1_final_position(2)^2)

d2_final_position
d2_distance = sqrt(d2_final_position(1)^2+d2_final_position(2)^2)

d3_final_position
d3_distance = sqrt(d3_final_position(1)^2+d3_final_position(2)^2)

d4_final_position
d4_distance = sqrt(d4_final_position(1)^2+d4_final_position(2)^2)

distance = [d1_distance d2_distance d3_distance d4_distance]

