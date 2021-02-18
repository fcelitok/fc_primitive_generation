%% In this program we are calculating primitive for given Ackermann vehicles
%
%function is working with max_steering_angle, car wheelbase and resolution

clear all;
close all;

steering_angle = 50;
wheelbase = 0.65;
resolution = 0.05;

max_dist = 5.0; 
numberOfAngle = 16; % only working with 16 angles
numberOfIntermediatePose = 10;

steering_angle_max = steering_angle*pi/180; % in radiants
min_radius = wheelbase/tan(steering_angle_max);


forward_move = 1;
bacward_move = 1;

% forward primitives costs
forward_penalty = 2; 
long_forward_penalty = 1;
curve_forward_penalty = 1;

% bacward primitives costs
bacward_penalty = 3; 
long_bacward_penalty = 2;
curve_bacward_penalty = 2;

print_stuff=1;

if(print_stuff==1)
    figure();
    hold on;
    grid on;
end

primID = 1;

start_position = [0 0];
for_loop_search = [resolution:resolution:max_dist];

% Straight lines x-y points
k = 2;
y = [0:k-1 k*ones(1,2*k+1) k-1:-1:-k+1 -k*ones(1,2*k+1) -k+1:-1];
x = -[-k*ones(1,k) [-k:k-1] k*ones(1,2*k+1) k-1:-1:-k -k*ones(1,k-1)];

% 16 number of angles
angles = [atan2(0,1) atan2(1,2) atan2(1,1) atan2(2,1) atan2(1,0) atan2(2,-1) atan2(1,-1) atan2(1,-2) atan2(0,-1) atan2(-1,-2) atan2(-1,-1) atan2(-2,-1) atan2(-1,0) atan2(-2,1) atan2(-1,1) atan2(-1,2)];

% atan2(0,1) direction 0
% atan2(1,2  direction 1                        6   5   4   3   2
% atan2(1,1)  direction 2                               |       
% atan2(2,1) direction 3                        7       |       1
% atan2(1 0)  direction 4                               |
% atan(2,-1) direction 5                        8 ------|------ 0                                           
% atan2(1, -1) direction 6                              |       
% atan2(1,-2) direction 7                       9       |      15  
% atan2(0, -1) direction 8                              |       
% atan2(-1,-2) direction 9                      10 11  12  13  14
% atan2(-1 -1)  direction 10                            
% atan2(-2,-1)  direction 11                   
% atan2(-1,0) direction 12
% atan2(-2,1) direction 13
% atan2(-1,1) direction 14
% atan2(-1,2) direction 15



%% forward move start-----------------------------------------------------------------------------------
if (forward_move ==1 ) 
degree = 1;

while degree <= 16
    if (degree == 2 || degree == 4 || degree == 6 || degree == 8 || degree == 10 || degree == 12 || degree == 14 || degree == 16)
        if(print_stuff==1)
        hold on;
        grid on;
        plot(x(degree)*resolution, y(degree)*resolution, 'r*')
        end
        straight_start_pose_theta = angles(degree);
        straight_start_pose = [start_position straight_start_pose_theta];

        end_position = [x(degree)*resolution y(degree)*resolution];
        straight_end_pose_theta = straight_start_pose_theta;
        straight_end_pose = [end_position straight_end_pose_theta];

        [clotoid_check_straight, p_straight, L ] = clotoide(straight_start_pose, straight_end_pose, numberOfIntermediatePose, min_radius);

        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose, p_straight, primID, resolution, forward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1);
        
        degree = degree + 1;
    else
        if(print_stuff==1)
        hold on;
        grid on;
        plot(x(degree)*(resolution/2), y(degree)*(resolution/2), 'r*')
        end    
        straight_start_pose_theta = angles(degree);
        straight_start_pose = [start_position straight_start_pose_theta];

        end_position = [x(degree)*(resolution/2) y(degree)*(resolution/2)];
        straight_end_pose_theta = straight_start_pose_theta;
        straight_end_pose = [end_position straight_end_pose_theta];

        [clotoid_check_straight, p_straight, L ] = clotoide(straight_start_pose, straight_end_pose, numberOfIntermediatePose, min_radius);
        
        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose, p_straight, primID, resolution, forward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1);
        
        degree = degree + 1; 
    end
end

%% For loop Quadrants:

for degree_number = 1:16
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    if (degree_number == 1 || degree_number == 2 || degree_number == 3 || degree_number == 4) %% Quadrant 1
        
        start_pose_theta = angles(degree_number+1);
        start_pose = [start_position start_pose_theta];
        if (degree_number ~= 1)
        end_pose_theta = angles(degree_number-1);

        start_pose_theta_2 = angles(degree_number-1);    
        start_pose_2 = [start_position start_pose_theta_2];
        end
        end_pose_theta_2 = angles(degree_number+1);
        
        if (degree_number == 1)
        % d1
        end_pose_theta = angles(16);

        start_pose_theta_2 = angles(16);    
        start_pose_2 = [start_position start_pose_theta_2];
        
        for x_position = for_loop_search
            final_position = [x_position 0];
            end_pose = [final_position end_pose_theta];
            [clotoid_check, p, L ] = clotoide(start_pose, end_pose, numberOfIntermediatePose, min_radius);

            x_position_2 = x_position;
            final_position_2 = [x_position_2 0];
            end_pose_2 = [final_position_2 end_pose_theta_2];
            [clotoid_check_2, p_2, L ] = clotoide(start_pose_2, end_pose_2, numberOfIntermediatePose, min_radius);

            if (clotoid_check == true && clotoid_check_2 == true)
                [primID, send_primitives] = primitive_function(start_pose, end_pose, p, start_pose_2, end_pose_2, p_2, primID, resolution, curve_forward_penalty, curve_forward_penalty, angles, print_stuff);
                primitives(primID-2) = send_primitives(primID-2);
                primitives(primID-1) = send_primitives(primID-1);
                break 
            end   
        end
        straight_start_pose_theta = angles(degree_number);
        straight_start_pose = [start_position straight_start_pose_theta];

        straight_end_pose_long = [end_pose(1) end_pose(2) angles(degree_number)];
        [clotoid_check_straight_long, p_straight_long, L ] = clotoide(straight_start_pose, straight_end_pose_long, numberOfIntermediatePose, min_radius);
        
        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose_long, p_straight_long, primID, resolution, long_forward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1);
        end

        % d2 and d3
        if(degree_number == 2)
        for y_position = for_loop_search
            final_position = [y_position*2 y_position];
            end_pose = [final_position end_pose_theta];
            [clotoid_check, p, L ] = clotoide(start_pose, end_pose, numberOfIntermediatePose, min_radius);

            y_position_2 = y_position;
            final_position_2 = [y_position_2*2 y_position_2];
            end_pose_2 = [final_position_2 end_pose_theta_2];
            [clotoid_check_2, p_2, L ] = clotoide(start_pose_2, end_pose_2, numberOfIntermediatePose, min_radius);

            if (clotoid_check == true && clotoid_check_2 == true)
                [primID, send_primitives] = primitive_function(start_pose, end_pose, p, start_pose_2, end_pose_2, p_2, primID, resolution, curve_forward_penalty, curve_forward_penalty, angles, print_stuff);
                primitives(primID-2) = send_primitives(primID-2);
                primitives(primID-1) = send_primitives(primID-1);
                break
            end 
        end
        straight_start_pose_theta = angles(degree_number);
        straight_start_pose = [start_position straight_start_pose_theta];

        straight_end_pose_long = [end_pose(1) end_pose(2) angles(degree_number)];
        [clotoid_check_straight_long, p_straight_long, L ] = clotoide(straight_start_pose, straight_end_pose_long, numberOfIntermediatePose, min_radius);
        
        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose_long, p_straight_long, primID, resolution, long_forward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1);
        end

        % d4 
        if(degree_number == 3)     
        for x_position = for_loop_search
            y_position = x_position;
            final_position = [x_position y_position];
            end_pose = [final_position end_pose_theta];
            [clotoid_check, p, L ] = clotoide(start_pose, end_pose, numberOfIntermediatePose, min_radius);

            x_position_2 = x_position;
            y_position_2 = x_position_2;
            final_position_2 = [x_position_2 y_position_2];
            end_pose_2 = [final_position_2 end_pose_theta_2];
            [clotoid_check_2, p_2, L ] = clotoide(start_pose_2, end_pose_2, numberOfIntermediatePose, min_radius);

            if (clotoid_check == true && clotoid_check_2 == true)
                [primID, send_primitives] = primitive_function(start_pose, end_pose, p, start_pose_2, end_pose_2, p_2, primID, resolution, curve_forward_penalty, curve_forward_penalty, angles, print_stuff);
                primitives(primID-2) = send_primitives(primID-2);
                primitives(primID-1) = send_primitives(primID-1);
                break
            end 
        end
        straight_start_pose_theta = angles(degree_number);
        straight_start_pose = [start_position straight_start_pose_theta];

        straight_end_pose_long = [end_pose(1) end_pose(2) angles(degree_number)];
        [clotoid_check_straight_long, p_straight_long, L ] = clotoide(straight_start_pose, straight_end_pose_long, numberOfIntermediatePose, min_radius);
        
        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose_long, p_straight_long, primID, resolution, long_forward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1);
        end
        
        %  d5 and d6
        if(degree_number == 4)
        for x_position = for_loop_search
            final_position = [x_position x_position*2];
            end_pose = [final_position end_pose_theta];
            [clotoid_check, p, L ] = clotoide(start_pose, end_pose, numberOfIntermediatePose, min_radius);

            x_position_2 = x_position;
            final_position_2 = [x_position_2 x_position_2*2];
            end_pose_2 = [final_position_2 end_pose_theta_2];
            [clotoid_check_2, p_2, L ] = clotoide(start_pose_2, end_pose_2, numberOfIntermediatePose, min_radius);

            if (clotoid_check == true && clotoid_check_2 == true)
                [primID, send_primitives] = primitive_function(start_pose, end_pose, p, start_pose_2, end_pose_2, p_2, primID, resolution, curve_forward_penalty, curve_forward_penalty, angles, print_stuff);
                primitives(primID-2) = send_primitives(primID-2);
                primitives(primID-1) = send_primitives(primID-1);
                break
            end 
        end
        straight_start_pose_theta = angles(degree_number);
        straight_start_pose = [start_position straight_start_pose_theta];

        straight_end_pose_long = [end_pose(1) end_pose(2) angles(degree_number)];
        [clotoid_check_straight_long, p_straight_long, L ] = clotoide(straight_start_pose, straight_end_pose_long, numberOfIntermediatePose, min_radius);
        
        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose_long, p_straight_long, primID, resolution, long_forward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1);
        end
     end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   if (degree_number == 5 || degree_number == 6 || degree_number == 7 || degree_number == 8) %% Quadrant 2
        
        start_pose_theta = angles(degree_number+1);
        start_pose = [start_position start_pose_theta];

        end_pose_theta = angles(degree_number-1);

        start_pose_theta_2 = angles(degree_number-1);    
        start_pose_2 = [start_position start_pose_theta_2];

        end_pose_theta_2 = angles(degree_number+1);
       
       if (degree_number == 5)
        % d1

        for y_position = for_loop_search
            final_position = [0 y_position];
            end_pose = [final_position end_pose_theta];
            [clotoid_check, p, L ] = clotoide(start_pose, end_pose, numberOfIntermediatePose, min_radius);

            y_position_2 = y_position;
            final_position_2 = [0 y_position_2];
            end_pose_2 = [final_position_2 end_pose_theta_2];
            [clotoid_check_2, p_2, L ] = clotoide(start_pose_2, end_pose_2, numberOfIntermediatePose, min_radius);

            if (clotoid_check == true && clotoid_check_2 == true)
                [primID, send_primitives] = primitive_function(start_pose, end_pose, p, start_pose_2, end_pose_2, p_2, primID, resolution, curve_forward_penalty, curve_forward_penalty, angles, print_stuff);
                primitives(primID-2) = send_primitives(primID-2);
                primitives(primID-1) = send_primitives(primID-1);
                break 
            end   
        end
        straight_start_pose_theta = angles(degree_number);
        straight_start_pose = [start_position straight_start_pose_theta];

        straight_end_pose_long = [end_pose(1) end_pose(2) angles(degree_number)];
        [clotoid_check_straight_long, p_straight_long, L ] = clotoide(straight_start_pose, straight_end_pose_long, numberOfIntermediatePose, min_radius);
        
        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose_long, p_straight_long, primID, resolution, long_forward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1); 
        end

        % d2 and d3
        if(degree_number == 6)
        for x_position = for_loop_search
            final_position = [-x_position x_position*2];
            end_pose = [final_position end_pose_theta];
            [clotoid_check, p, L ] = clotoide(start_pose, end_pose, numberOfIntermediatePose, min_radius);

            x_position_2 = x_position;
            final_position_2 = [-x_position_2 x_position_2*2];
            end_pose_2 = [final_position_2 end_pose_theta_2];
            [clotoid_check_2, p_2, L ] = clotoide(start_pose_2, end_pose_2, numberOfIntermediatePose, min_radius);

            if (clotoid_check == true && clotoid_check_2 == true)
                [primID, send_primitives] = primitive_function(start_pose, end_pose, p, start_pose_2, end_pose_2, p_2, primID, resolution, curve_forward_penalty, curve_forward_penalty, angles, print_stuff);
                primitives(primID-2) = send_primitives(primID-2);
                primitives(primID-1) = send_primitives(primID-1);
                break
            end 
        end
        straight_start_pose_theta = angles(degree_number);
        straight_start_pose = [start_position straight_start_pose_theta];

        straight_end_pose_long = [end_pose(1) end_pose(2) angles(degree_number)];
        [clotoid_check_straight_long, p_straight_long, L ] = clotoide(straight_start_pose, straight_end_pose_long, numberOfIntermediatePose, min_radius);
        
        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose_long, p_straight_long, primID, resolution, long_forward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1);
        end

        % d4 
        if(degree_number == 7)
        for x_position = for_loop_search
            y_position = x_position;
            final_position = [-x_position y_position];
            end_pose = [final_position end_pose_theta];
            [clotoid_check, p, L ] = clotoide(start_pose, end_pose, numberOfIntermediatePose, min_radius);

            x_position_2 = x_position;
            y_position_2 = x_position_2;
            final_position_2 = [-x_position_2 y_position_2];
            end_pose_2 = [final_position_2 end_pose_theta_2];
            [clotoid_check_2, p_2, L ] = clotoide(start_pose_2, end_pose_2, numberOfIntermediatePose, min_radius);

            if (clotoid_check == true && clotoid_check_2 == true)
                [primID, send_primitives] = primitive_function(start_pose, end_pose, p, start_pose_2, end_pose_2, p_2, primID, resolution, curve_forward_penalty, curve_forward_penalty, angles, print_stuff);
                primitives(primID-2) = send_primitives(primID-2);
                primitives(primID-1) = send_primitives(primID-1);
                break
            end 
        end
        straight_start_pose_theta = angles(degree_number);
        straight_start_pose = [start_position straight_start_pose_theta];

        straight_end_pose_long = [end_pose(1) end_pose(2) angles(degree_number)];
        [clotoid_check_straight_long, p_straight_long, L ] = clotoide(straight_start_pose, straight_end_pose_long, numberOfIntermediatePose, min_radius);
        
        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose_long, p_straight_long, primID, resolution, long_forward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1);
        end
       
        
        % d5 and d6
        if(degree_number == 8)
        for y_position = for_loop_search
            final_position = [-y_position*2 y_position];
            end_pose = [final_position end_pose_theta];
            [clotoid_check, p, L ] = clotoide(start_pose, end_pose, numberOfIntermediatePose, min_radius);

            y_position_2 = y_position;
            final_position_2 = [-y_position_2*2 y_position_2];
            end_pose_2 = [final_position_2 end_pose_theta_2];
            [clotoid_check_2, p_2, L ] = clotoide(start_pose_2, end_pose_2, numberOfIntermediatePose, min_radius);

            if (clotoid_check == true && clotoid_check_2 == true)
                [primID, send_primitives] = primitive_function(start_pose, end_pose, p, start_pose_2, end_pose_2, p_2, primID, resolution, curve_forward_penalty, curve_forward_penalty, angles, print_stuff);
                primitives(primID-2) = send_primitives(primID-2);
                primitives(primID-1) = send_primitives(primID-1);
                break
            end 
        end
        straight_start_pose_theta = angles(degree_number);
        straight_start_pose = [start_position straight_start_pose_theta];

        straight_end_pose_long = [end_pose(1) end_pose(2) angles(degree_number)];
        [clotoid_check_straight_long, p_straight_long, L ] = clotoide(straight_start_pose, straight_end_pose_long, numberOfIntermediatePose, min_radius);
        
        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose_long, p_straight_long, primID, resolution, long_forward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1);
        end    
    end
    
    
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    if (degree_number == 9 || degree_number == 10 || degree_number == 11 || degree_number == 12) %% Quadrant 3
        
        start_pose_theta = angles(degree_number+1);
        start_pose = [start_position start_pose_theta];
        
        end_pose_theta = angles(degree_number-1);

        start_pose_theta_2 = angles(degree_number-1);    
        start_pose_2 = [start_position start_pose_theta_2];
        
        end_pose_theta_2 = angles(degree_number+1);
        
        % d1 
        if (degree_number == 9)       
        for x_position = for_loop_search
            final_position = [-x_position 0];
            end_pose = [final_position end_pose_theta];
            [clotoid_check, p, L ] = clotoide(start_pose, end_pose, numberOfIntermediatePose, min_radius);

            x_position_2 = x_position;
            final_position_2 = [-x_position_2 0];
            end_pose_2 = [final_position_2 end_pose_theta_2];
            [clotoid_check_2, p_2, L ] = clotoide(start_pose_2, end_pose_2, numberOfIntermediatePose, min_radius);

            if (clotoid_check == true && clotoid_check_2 == true)
                [primID, send_primitives] = primitive_function(start_pose, end_pose, p, start_pose_2, end_pose_2, p_2, primID, resolution, curve_forward_penalty, curve_forward_penalty, angles, print_stuff);
                primitives(primID-2) = send_primitives(primID-2);
                primitives(primID-1) = send_primitives(primID-1);
                break 
            end   
        end
        straight_start_pose_theta = angles(degree_number);
        straight_start_pose = [start_position straight_start_pose_theta];

        straight_end_pose_long = [end_pose(1) end_pose(2) angles(degree_number)];
        [clotoid_check_straight_long, p_straight_long, L ] = clotoide(straight_start_pose, straight_end_pose_long, numberOfIntermediatePose, min_radius);
        
        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose_long, p_straight_long, primID, resolution, long_forward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1);
        end

        % d2 and d3
        if(degree_number == 10)
        for y_position = for_loop_search
            final_position = [-y_position*2 -y_position];
            end_pose = [final_position end_pose_theta];
            [clotoid_check, p, L ] = clotoide(start_pose, end_pose, numberOfIntermediatePose, min_radius);

            y_position_2 = y_position;
            final_position_2 = [-y_position_2*2 -y_position_2];
            end_pose_2 = [final_position_2 end_pose_theta_2];
            [clotoid_check_2, p_2, L ] = clotoide(start_pose_2, end_pose_2, numberOfIntermediatePose, min_radius);

            if (clotoid_check == true && clotoid_check_2 == true)
                [primID, send_primitives] = primitive_function(start_pose, end_pose, p, start_pose_2, end_pose_2, p_2, primID, resolution, curve_forward_penalty, curve_forward_penalty, angles, print_stuff);
                primitives(primID-2) = send_primitives(primID-2);
                primitives(primID-1) = send_primitives(primID-1);
                break
            end 
        end
        straight_start_pose_theta = angles(degree_number);
        straight_start_pose = [start_position straight_start_pose_theta];

        straight_end_pose_long = [end_pose(1) end_pose(2) angles(degree_number)];
        [clotoid_check_straight_long, p_straight_long, L ] = clotoide(straight_start_pose, straight_end_pose_long, numberOfIntermediatePose, min_radius);
        
        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose_long, p_straight_long, primID, resolution, long_forward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1);
        end

        % d4 
        if(degree_number == 11)     
        for x_position = for_loop_search
            y_position = x_position;
            final_position = [-x_position -y_position];
            end_pose = [final_position end_pose_theta];
            [clotoid_check, p, L ] = clotoide(start_pose, end_pose, numberOfIntermediatePose, min_radius);

            x_position_2 = x_position;
            y_position_2 = x_position_2;
            final_position_2 = [-x_position_2 -y_position_2];
            end_pose_2 = [final_position_2 end_pose_theta_2];
            [clotoid_check_2, p_2, L ] = clotoide(start_pose_2, end_pose_2, numberOfIntermediatePose, min_radius);

            if (clotoid_check == true && clotoid_check_2 == true)
                [primID, send_primitives] = primitive_function(start_pose, end_pose, p, start_pose_2, end_pose_2, p_2, primID, resolution, curve_forward_penalty, curve_forward_penalty, angles, print_stuff);
                primitives(primID-2) = send_primitives(primID-2);
                primitives(primID-1) = send_primitives(primID-1);
                break
            end 
        end
        straight_start_pose_theta = angles(degree_number);
        straight_start_pose = [start_position straight_start_pose_theta];

        straight_end_pose_long = [end_pose(1) end_pose(2) angles(degree_number)];
        [clotoid_check_straight_long, p_straight_long, L ] = clotoide(straight_start_pose, straight_end_pose_long, numberOfIntermediatePose, min_radius);
        
        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose_long, p_straight_long, primID, resolution, long_forward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1);
        end
        
        %  d5 and d6
        if(degree_number == 12)
        for x_position = for_loop_search
            final_position = [-x_position -x_position*2];
            end_pose = [final_position end_pose_theta];
            [clotoid_check, p, L ] = clotoide(start_pose, end_pose, numberOfIntermediatePose, min_radius);

            x_position_2 = x_position;
            final_position_2 = [-x_position_2 -x_position_2*2];
            end_pose_2 = [final_position_2 end_pose_theta_2];
            [clotoid_check_2, p_2, L ] = clotoide(start_pose_2, end_pose_2, numberOfIntermediatePose, min_radius);

            if (clotoid_check == true && clotoid_check_2 == true)
                [primID, send_primitives] = primitive_function(start_pose, end_pose, p, start_pose_2, end_pose_2, p_2, primID, resolution, curve_forward_penalty, curve_forward_penalty, angles, print_stuff);
                primitives(primID-2) = send_primitives(primID-2);
                primitives(primID-1) = send_primitives(primID-1);
                break
            end 
        end
        straight_start_pose_theta = angles(degree_number);
        straight_start_pose = [start_position straight_start_pose_theta];

        straight_end_pose_long = [end_pose(1) end_pose(2) angles(degree_number)];
        [clotoid_check_straight_long, p_straight_long, L ] = clotoide(straight_start_pose, straight_end_pose_long, numberOfIntermediatePose, min_radius);
        
        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose_long, p_straight_long, primID, resolution, long_forward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1);
        end
     end
    
    
    
% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    if (degree_number == 13 || degree_number == 14 || degree_number == 15 || degree_number == 16) %% Quadrant 4
        if ( degree_number ~= 16)
        start_pose_theta = angles(degree_number+1);
        start_pose = [start_position start_pose_theta];

        end_pose_theta = angles(degree_number-1);

        start_pose_theta_2 = angles(degree_number-1);    
        start_pose_2 = [start_position start_pose_theta_2];

        end_pose_theta_2 = angles(degree_number+1);
        end
        % d1
        if (degree_number == 13)
        for y_position = for_loop_search
            final_position = [0 -y_position];
            end_pose = [final_position end_pose_theta];
            [clotoid_check, p, L ] = clotoide(start_pose, end_pose, numberOfIntermediatePose, min_radius);

            y_position_2 = y_position;
            final_position_2 = [0 -y_position_2];
            end_pose_2 = [final_position_2 end_pose_theta_2];
            [clotoid_check_2, p_2, L ] = clotoide(start_pose_2, end_pose_2, numberOfIntermediatePose, min_radius);

            if (clotoid_check == true && clotoid_check_2 == true)
                [primID, send_primitives] = primitive_function(start_pose, end_pose, p, start_pose_2, end_pose_2, p_2, primID, resolution, curve_forward_penalty, curve_forward_penalty, angles, print_stuff);
                primitives(primID-2) = send_primitives(primID-2);
                primitives(primID-1) = send_primitives(primID-1);
                break 
            end   
        end
        straight_start_pose_theta = angles(degree_number);
        straight_start_pose = [start_position straight_start_pose_theta];

        straight_end_pose_long = [end_pose(1) end_pose(2) angles(degree_number)];
        [clotoid_check_straight_long, p_straight_long, L ] = clotoide(straight_start_pose, straight_end_pose_long, numberOfIntermediatePose, min_radius);
        
        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose_long, p_straight_long, primID, resolution, long_forward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1);
        end

        % d2 and d3
        if(degree_number == 14)
        for x_position = for_loop_search
            final_position = [x_position -x_position*2];
            end_pose = [final_position end_pose_theta];
            [clotoid_check, p, L ] = clotoide(start_pose, end_pose, numberOfIntermediatePose, min_radius);

            x_position_2 = x_position;
            final_position_2 = [x_position_2 -x_position_2*2];
            end_pose_2 = [final_position_2 end_pose_theta_2];
            [clotoid_check_2, p_2, L ] = clotoide(start_pose_2, end_pose_2, numberOfIntermediatePose, min_radius);

            if (clotoid_check == true && clotoid_check_2 == true)
                [primID, send_primitives] = primitive_function(start_pose, end_pose, p, start_pose_2, end_pose_2, p_2, primID, resolution, curve_forward_penalty, curve_forward_penalty, angles, print_stuff);
                primitives(primID-2) = send_primitives(primID-2);
                primitives(primID-1) = send_primitives(primID-1);
                break
            end 
        end
        straight_start_pose_theta = angles(degree_number);
        straight_start_pose = [start_position straight_start_pose_theta];

        straight_end_pose_long = [end_pose(1) end_pose(2) angles(degree_number)];
        [clotoid_check_straight_long, p_straight_long, L ] = clotoide(straight_start_pose, straight_end_pose_long, numberOfIntermediatePose, min_radius);
        
        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose_long, p_straight_long, primID, resolution, long_forward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1);
        end

        % d4 
        if(degree_number == 15)
        for x_position = for_loop_search
            y_position = x_position;
            final_position = [x_position -y_position];
            end_pose = [final_position end_pose_theta];
            [clotoid_check, p, L ] = clotoide(start_pose, end_pose, numberOfIntermediatePose, min_radius);

            x_position_2 = x_position;
            y_position_2 = x_position_2;
            final_position_2 = [x_position_2 -y_position_2];
            end_pose_2 = [final_position_2 end_pose_theta_2];
            [clotoid_check_2, p_2, L ] = clotoide(start_pose_2, end_pose_2, numberOfIntermediatePose, min_radius);

            if (clotoid_check == true && clotoid_check_2 == true)
                [primID, send_primitives] = primitive_function(start_pose, end_pose, p, start_pose_2, end_pose_2, p_2, primID, resolution, curve_forward_penalty, curve_forward_penalty, angles, print_stuff);
                primitives(primID-2) = send_primitives(primID-2);
                primitives(primID-1) = send_primitives(primID-1);
                break
            end 
        end
        straight_start_pose_theta = angles(degree_number);
        straight_start_pose = [start_position straight_start_pose_theta];

        straight_end_pose_long = [end_pose(1) end_pose(2) angles(degree_number)];
        [clotoid_check_straight_long, p_straight_long, L ] = clotoide(straight_start_pose, straight_end_pose_long, numberOfIntermediatePose, min_radius);
        
        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose_long, p_straight_long, primID, resolution, long_forward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1);
        end
       
        
        % d5 and d6
        if(degree_number == 16)
            start_pose_theta = angles(1);
            start_pose = [start_position start_pose_theta];

            end_pose_theta = angles(degree_number-1);

            start_pose_theta_2 = angles(degree_number-1);    
            start_pose_2 = [start_position start_pose_theta_2];

            end_pose_theta_2 = angles(1);
            
        for y_position = for_loop_search
            final_position = [y_position*2 -y_position];
            end_pose = [final_position end_pose_theta];
            [clotoid_check, p, L ] = clotoide(start_pose, end_pose, numberOfIntermediatePose, min_radius);

            y_position_2 = y_position;
            final_position_2 = [y_position_2*2 -y_position_2];
            end_pose_2 = [final_position_2 end_pose_theta_2];
            [clotoid_check_2, p_2, L ] = clotoide(start_pose_2, end_pose_2, numberOfIntermediatePose, min_radius);

            if (clotoid_check == true && clotoid_check_2 == true)
                [primID, send_primitives] = primitive_function(start_pose, end_pose, p, start_pose_2, end_pose_2, p_2, primID, resolution, curve_forward_penalty, curve_forward_penalty, angles, print_stuff);
                primitives(primID-2) = send_primitives(primID-2);
                primitives(primID-1) = send_primitives(primID-1);
                break
            end 
        end
        straight_start_pose_theta = angles(degree_number);
        straight_start_pose = [start_position straight_start_pose_theta];

        straight_end_pose_long = [end_pose(1) end_pose(2) angles(degree_number)];
        [clotoid_check_straight_long, p_straight_long, L ] = clotoide(straight_start_pose, straight_end_pose_long, numberOfIntermediatePose, min_radius);
        
        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose_long, p_straight_long, primID, resolution, long_forward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1);
        end    
    end
    
end

end  %% forward move end-----------------------------------------------------------------------------------


%% bacward move start-----------------------------------------------------------------------------------
if (bacward_move ==1 ) 
degree = 1;

while degree <= 16
    if (degree == 2 || degree == 4 || degree == 6 || degree == 8 || degree == 10 || degree == 12 || degree == 14 || degree == 16)
        if(print_stuff==1)
        hold on;
        grid on;
        plot(-x(degree)*resolution, -y(degree)*resolution, 'r*')
        end
        straight_start_pose_theta = angles(degree);
        straight_start_pose = [start_position straight_start_pose_theta];

        end_position = [-x(degree)*resolution -y(degree)*resolution];
        straight_end_pose_theta = straight_start_pose_theta;
        straight_end_pose = [end_position straight_end_pose_theta];

        [clotoid_check_straight, f_p_straight, L ] = clotoide(straight_end_pose, straight_start_pose, numberOfIntermediatePose, min_radius);
        
        p_straight = flip(f_p_straight);
        
        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose, p_straight, primID, resolution, bacward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1);
        
        degree = degree + 1;
    else
        if(print_stuff==1)
        hold on;
        grid on;
        plot(-x(degree)*(resolution/2), -y(degree)*(resolution/2), 'r*')
        end    
        straight_start_pose_theta = angles(degree);
        straight_start_pose = [start_position straight_start_pose_theta];

        end_position = [-x(degree)*(resolution/2) -y(degree)*(resolution/2)];
        straight_end_pose_theta = straight_start_pose_theta;
        straight_end_pose = [end_position straight_end_pose_theta];
        
        [clotoid_check_straight, f_p_straight, L ] = clotoide(straight_end_pose, straight_start_pose, numberOfIntermediatePose, min_radius);
        
        p_straight = flip(f_p_straight);
                
        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose, p_straight, primID, resolution, bacward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1);
        
        degree = degree + 1; 
    end
end

%% For loop Quadrants:

for degree_number = 1:16
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    if (degree_number == 1 || degree_number == 2 || degree_number == 3 || degree_number == 4) %% Quadrant 1
        
        start_pose_theta = angles(degree_number+1);
        start_pose = [start_position start_pose_theta];
        if (degree_number ~= 1)
        end_pose_theta = angles(degree_number-1);

        start_pose_theta_2 = angles(degree_number-1);    
        start_pose_2 = [start_position start_pose_theta_2];
        end
        end_pose_theta_2 = angles(degree_number+1);
        
        if (degree_number == 1)
        % d1
        end_pose_theta = angles(16);

        start_pose_theta_2 = angles(16);    
        start_pose_2 = [start_position start_pose_theta_2];
        
        for x_position = for_loop_search
            final_position = [-x_position 0];
            end_pose = [final_position end_pose_theta];
            [clotoid_check, f_p, L ] = clotoide(end_pose, start_pose, numberOfIntermediatePose, min_radius);

            x_position_2 = x_position;
            final_position_2 = [-x_position_2 0];
            end_pose_2 = [final_position_2 end_pose_theta_2];
            [clotoid_check_2, f_p_2, L ] = clotoide(end_pose_2, start_pose_2, numberOfIntermediatePose, min_radius);
            
            p = flip(f_p);
            p_2= flip(f_p_2);

            if (clotoid_check == true && clotoid_check_2 == true)
                [primID, send_primitives] = primitive_function(start_pose, end_pose, p, start_pose_2, end_pose_2, p_2, primID, resolution, curve_bacward_penalty, curve_bacward_penalty, angles, print_stuff);
                primitives(primID-2) = send_primitives(primID-2);
                primitives(primID-1) = send_primitives(primID-1);
                break 
            end   
        end
        straight_start_pose_theta = angles(degree_number);
        straight_start_pose = [start_position straight_start_pose_theta];

        straight_end_pose_long = [end_pose(1) end_pose(2) angles(degree_number)];
        [clotoid_check_straight_long, f_p_straight_long, L ] = clotoide(straight_end_pose_long, straight_start_pose, numberOfIntermediatePose, min_radius);
        
        p_straight_long = flip(f_p_straight_long);
        
        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose_long, p_straight_long, primID, resolution, long_bacward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1);
        end

        % d2 and d3
        if(degree_number == 2)
        for y_position = for_loop_search
            final_position = [-y_position*2 -y_position];
            end_pose = [final_position end_pose_theta];
            [clotoid_check, f_p, L ] = clotoide(end_pose, start_pose, numberOfIntermediatePose, min_radius);

            y_position_2 = y_position;
            final_position_2 = [-y_position_2*2 -y_position_2];
            end_pose_2 = [final_position_2 end_pose_theta_2];
            [clotoid_check_2, f_p_2, L ] = clotoide(end_pose_2, start_pose_2, numberOfIntermediatePose, min_radius);
            
            p = flip(f_p);
            p_2= flip(f_p_2);
            
            if (clotoid_check == true && clotoid_check_2 == true)
                [primID, send_primitives] = primitive_function(start_pose, end_pose, p, start_pose_2, end_pose_2, p_2, primID, resolution, curve_bacward_penalty, curve_bacward_penalty, angles, print_stuff);
                primitives(primID-2) = send_primitives(primID-2);
                primitives(primID-1) = send_primitives(primID-1);
                break
            end 
        end
        straight_start_pose_theta = angles(degree_number);
        straight_start_pose = [start_position straight_start_pose_theta];

        straight_end_pose_long = [end_pose(1) end_pose(2) angles(degree_number)];
        [clotoid_check_straight_long, f_p_straight_long, L ] = clotoide(straight_end_pose_long, straight_start_pose, numberOfIntermediatePose, min_radius);
        
        p_straight_long = flip(f_p_straight_long);
        
        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose_long, p_straight_long, primID, resolution, long_bacward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1);
        end

        % d4 
        if(degree_number == 3)     
        for x_position = for_loop_search
            y_position = x_position;
            final_position = [-x_position -y_position];
            end_pose = [final_position end_pose_theta];
            [clotoid_check, f_p, L ] = clotoide(end_pose, start_pose, numberOfIntermediatePose, min_radius);

            x_position_2 = x_position;
            y_position_2 = x_position_2;
            final_position_2 = [-x_position_2 -y_position_2];
            end_pose_2 = [final_position_2 end_pose_theta_2];
            [clotoid_check_2, f_p_2, L ] = clotoide(end_pose_2, start_pose_2, numberOfIntermediatePose, min_radius);
            
            p = flip(f_p);
            p_2= flip(f_p_2);
            
            if (clotoid_check == true && clotoid_check_2 == true)
                [primID, send_primitives] = primitive_function(start_pose, end_pose, p, start_pose_2, end_pose_2, p_2, primID, resolution, curve_bacward_penalty, curve_bacward_penalty, angles, print_stuff);
                primitives(primID-2) = send_primitives(primID-2);
                primitives(primID-1) = send_primitives(primID-1);
                break
            end 
        end
        straight_start_pose_theta = angles(degree_number);
        straight_start_pose = [start_position straight_start_pose_theta];

        straight_end_pose_long = [end_pose(1) end_pose(2) angles(degree_number)];
        [clotoid_check_straight_long, f_p_straight_long, L ] = clotoide(straight_end_pose_long, straight_start_pose, numberOfIntermediatePose, min_radius);
        
        p_straight_long = flip(f_p_straight_long);
        
        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose_long, p_straight_long, primID, resolution, long_bacward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1);
        end
        
        %  d5 and d6
        if(degree_number == 4)
        for x_position = for_loop_search
            final_position = [-x_position -x_position*2];
            end_pose = [final_position end_pose_theta];
            [clotoid_check, f_p, L ] = clotoide(end_pose, start_pose, numberOfIntermediatePose, min_radius);

            x_position_2 = x_position;
            final_position_2 = [-x_position_2 -x_position_2*2];
            end_pose_2 = [final_position_2 end_pose_theta_2];
            [clotoid_check_2, f_p_2, L ] = clotoide(end_pose_2, start_pose_2, numberOfIntermediatePose, min_radius);
            
            p = flip(f_p);
            p_2= flip(f_p_2);
            
            if (clotoid_check == true && clotoid_check_2 == true)
                [primID, send_primitives] = primitive_function(start_pose, end_pose, p, start_pose_2, end_pose_2, p_2, primID, resolution, curve_bacward_penalty, curve_bacward_penalty, angles, print_stuff);
                primitives(primID-2) = send_primitives(primID-2);
                primitives(primID-1) = send_primitives(primID-1);
                break
            end 
        end
        straight_start_pose_theta = angles(degree_number);
        straight_start_pose = [start_position straight_start_pose_theta];

        straight_end_pose_long = [end_pose(1) end_pose(2) angles(degree_number)];
        [clotoid_check_straight_long, f_p_straight_long, L ] = clotoide(straight_end_pose_long, straight_start_pose, numberOfIntermediatePose, min_radius);
        
        p_straight_long = flip(f_p_straight_long);
        
        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose_long, p_straight_long, primID, resolution, long_bacward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1);
        end
     end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   if (degree_number == 5 || degree_number == 6 || degree_number == 7 || degree_number == 8) %% Quadrant 2
        
        start_pose_theta = angles(degree_number+1);
        start_pose = [start_position start_pose_theta];

        end_pose_theta = angles(degree_number-1);

        start_pose_theta_2 = angles(degree_number-1);    
        start_pose_2 = [start_position start_pose_theta_2];

        end_pose_theta_2 = angles(degree_number+1);
       
       if (degree_number == 5)
        % d1

        for y_position = for_loop_search
            final_position = [0 -y_position];
            end_pose = [final_position end_pose_theta];
            [clotoid_check, f_p, L ] = clotoide(end_pose, start_pose, numberOfIntermediatePose, min_radius);

            y_position_2 = y_position;
            final_position_2 = [0 -y_position_2];
            end_pose_2 = [final_position_2 end_pose_theta_2];
            [clotoid_check_2, f_p_2, L ] = clotoide(end_pose_2, start_pose_2, numberOfIntermediatePose, min_radius);
            
            p = flip(f_p);
            p_2= flip(f_p_2);

            if (clotoid_check == true && clotoid_check_2 == true)
                [primID, send_primitives] = primitive_function(start_pose, end_pose, p, start_pose_2, end_pose_2, p_2, primID, resolution, curve_bacward_penalty, curve_bacward_penalty, angles, print_stuff);
                primitives(primID-2) = send_primitives(primID-2);
                primitives(primID-1) = send_primitives(primID-1);
                break 
            end   
        end
        straight_start_pose_theta = angles(degree_number);
        straight_start_pose = [start_position straight_start_pose_theta];

        straight_end_pose_long = [end_pose(1) end_pose(2) angles(degree_number)];
        [clotoid_check_straight_long, f_p_straight_long, L ] = clotoide(straight_end_pose_long, straight_start_pose, numberOfIntermediatePose, min_radius);
        
        p_straight_long = flip(f_p_straight_long);
        
        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose_long, p_straight_long, primID, resolution, long_bacward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1);
        end

        % d2 and d3
        if(degree_number == 6)
        for x_position = for_loop_search
            final_position = [x_position -x_position*2];
            end_pose = [final_position end_pose_theta];
            [clotoid_check, f_p, L ] = clotoide(end_pose, start_pose, numberOfIntermediatePose, min_radius);

            x_position_2 = x_position;
            final_position_2 = [x_position_2 -x_position_2*2];
            end_pose_2 = [final_position_2 end_pose_theta_2];
            [clotoid_check_2, f_p_2, L ] = clotoide(end_pose_2, start_pose_2, numberOfIntermediatePose, min_radius);
            
            p = flip(f_p);
            p_2= flip(f_p_2);
            
            if (clotoid_check == true && clotoid_check_2 == true)
                [primID, send_primitives] = primitive_function(start_pose, end_pose, p, start_pose_2, end_pose_2, p_2, primID, resolution, curve_bacward_penalty, curve_bacward_penalty, angles, print_stuff);
                primitives(primID-2) = send_primitives(primID-2);
                primitives(primID-1) = send_primitives(primID-1);
                break
            end 
        end 
        straight_start_pose_theta = angles(degree_number);
        straight_start_pose = [start_position straight_start_pose_theta];

        straight_end_pose_long = [end_pose(1) end_pose(2) angles(degree_number)];
        [clotoid_check_straight_long, f_p_straight_long, L ] = clotoide(straight_end_pose_long, straight_start_pose, numberOfIntermediatePose, min_radius);
        
        p_straight_long = flip(f_p_straight_long);
        
        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose_long, p_straight_long, primID, resolution, long_bacward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1);
        end

        % d4 
        if(degree_number == 7)
        for x_position = for_loop_search
            y_position = x_position;
            final_position = [x_position -y_position];
            end_pose = [final_position end_pose_theta];
            [clotoid_check, f_p, L ] = clotoide(end_pose, start_pose, numberOfIntermediatePose, min_radius);

            x_position_2 = x_position;
            y_position_2 = x_position_2;
            final_position_2 = [x_position_2 -y_position_2];
            end_pose_2 = [final_position_2 end_pose_theta_2];
            [clotoid_check_2, f_p_2, L ] = clotoide(end_pose_2, start_pose_2, numberOfIntermediatePose, min_radius);
            
            p = flip(f_p);
            p_2= flip(f_p_2);
            
            if (clotoid_check == true && clotoid_check_2 == true)
                [primID, send_primitives] = primitive_function(start_pose, end_pose, p, start_pose_2, end_pose_2, p_2, primID, resolution, curve_bacward_penalty, curve_bacward_penalty, angles, print_stuff);
                primitives(primID-2) = send_primitives(primID-2);
                primitives(primID-1) = send_primitives(primID-1);
                break
            end 
        end
        straight_start_pose_theta = angles(degree_number);
        straight_start_pose = [start_position straight_start_pose_theta];

        straight_end_pose_long = [end_pose(1) end_pose(2) angles(degree_number)];
        [clotoid_check_straight_long, f_p_straight_long, L ] = clotoide(straight_end_pose_long, straight_start_pose, numberOfIntermediatePose, min_radius);
        
        p_straight_long = flip(f_p_straight_long);
        
        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose_long, p_straight_long, primID, resolution, long_bacward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1);
        end
       
        
        % d5 and d6
        if(degree_number == 8)
        for y_position = for_loop_search
            final_position = [y_position*2 -y_position];
            end_pose = [final_position end_pose_theta];
            [clotoid_check, f_p, L ] = clotoide(end_pose, start_pose, numberOfIntermediatePose, min_radius);

            y_position_2 = y_position;
            final_position_2 = [y_position_2*2 -y_position_2];
            end_pose_2 = [final_position_2 end_pose_theta_2];
            [clotoid_check_2, f_p_2, L ] = clotoide(end_pose_2, start_pose_2, numberOfIntermediatePose, min_radius);
            
            p = flip(f_p);
            p_2= flip(f_p_2);
            
            if (clotoid_check == true && clotoid_check_2 == true)
                [primID, send_primitives] = primitive_function(start_pose, end_pose, p, start_pose_2, end_pose_2, p_2, primID, resolution, curve_bacward_penalty, curve_bacward_penalty, angles, print_stuff);
                primitives(primID-2) = send_primitives(primID-2);
                primitives(primID-1) = send_primitives(primID-1);
                break
            end 
        end
        straight_start_pose_theta = angles(degree_number);
        straight_start_pose = [start_position straight_start_pose_theta];

        straight_end_pose_long = [end_pose(1) end_pose(2) angles(degree_number)];
        [clotoid_check_straight_long, f_p_straight_long, L ] = clotoide(straight_end_pose_long, straight_start_pose, numberOfIntermediatePose, min_radius);
        
        p_straight_long = flip(f_p_straight_long);
        
        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose_long, p_straight_long, primID, resolution, long_bacward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1);
        end    
    end
    
    
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    if (degree_number == 9 || degree_number == 10 || degree_number == 11 || degree_number == 12) %% Quadrant 3
        
        start_pose_theta = angles(degree_number+1);
        start_pose = [start_position start_pose_theta];
        
        end_pose_theta = angles(degree_number-1);

        start_pose_theta_2 = angles(degree_number-1);    
        start_pose_2 = [start_position start_pose_theta_2];
        
        end_pose_theta_2 = angles(degree_number+1);
        
        % d1 
        if (degree_number == 9)       
        for x_position = for_loop_search
            final_position = [x_position 0];
            end_pose = [final_position end_pose_theta];
            [clotoid_check, f_p, L ] = clotoide(end_pose, start_pose, numberOfIntermediatePose, min_radius);

            x_position_2 = x_position;
            final_position_2 = [x_position_2 0];
            end_pose_2 = [final_position_2 end_pose_theta_2];
            [clotoid_check_2, f_p_2, L ] = clotoide(end_pose_2, start_pose_2, numberOfIntermediatePose, min_radius);

            p = flip(f_p);
            p_2= flip(f_p_2);
            
            if (clotoid_check == true && clotoid_check_2 == true)
                [primID, send_primitives] = primitive_function(start_pose, end_pose, p, start_pose_2, end_pose_2, p_2, primID, resolution, curve_bacward_penalty, curve_bacward_penalty, angles, print_stuff);
                primitives(primID-2) = send_primitives(primID-2);
                primitives(primID-1) = send_primitives(primID-1);
                break 
            end   
        end
        straight_start_pose_theta = angles(degree_number);
        straight_start_pose = [start_position straight_start_pose_theta];

        straight_end_pose_long = [end_pose(1) end_pose(2) angles(degree_number)];
        [clotoid_check_straight_long, f_p_straight_long, L ] = clotoide(straight_end_pose_long, straight_start_pose, numberOfIntermediatePose, min_radius);
        
        p_straight_long = flip(f_p_straight_long);
        
        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose_long, p_straight_long, primID, resolution, long_bacward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1);
        end

        % d2 and d3
        if(degree_number == 10)
        for y_position = for_loop_search
            final_position = [y_position*2 y_position];
            end_pose = [final_position end_pose_theta];
            [clotoid_check, f_p, L ] = clotoide(end_pose, start_pose, numberOfIntermediatePose, min_radius);

            y_position_2 = y_position;
            final_position_2 = [y_position_2*2 y_position_2];
            end_pose_2 = [final_position_2 end_pose_theta_2];
            [clotoid_check_2, f_p_2, L ] = clotoide(end_pose_2, start_pose_2, numberOfIntermediatePose, min_radius);
            
            p = flip(f_p);
            p_2= flip(f_p_2);
            
            if (clotoid_check == true && clotoid_check_2 == true)
                [primID, send_primitives] = primitive_function(start_pose, end_pose, p, start_pose_2, end_pose_2, p_2, primID, resolution, curve_bacward_penalty, curve_bacward_penalty, angles, print_stuff);
                primitives(primID-2) = send_primitives(primID-2);
                primitives(primID-1) = send_primitives(primID-1);
                break
            end 
        end
        straight_start_pose_theta = angles(degree_number);
        straight_start_pose = [start_position straight_start_pose_theta];

        straight_end_pose_long = [end_pose(1) end_pose(2) angles(degree_number)];
        [clotoid_check_straight_long, f_p_straight_long, L ] = clotoide(straight_end_pose_long, straight_start_pose, numberOfIntermediatePose, min_radius);
        
        p_straight_long = flip(f_p_straight_long);
        
        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose_long, p_straight_long, primID, resolution, long_bacward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1);
        end

        % d4 
        if(degree_number == 11)     
        for x_position = for_loop_search
            y_position = x_position;
            final_position = [x_position y_position];
            end_pose = [final_position end_pose_theta];
            [clotoid_check, f_p, L ] = clotoide(end_pose, start_pose, numberOfIntermediatePose, min_radius);

            x_position_2 = x_position;
            y_position_2 = x_position_2;
            final_position_2 = [x_position_2 y_position_2];
            end_pose_2 = [final_position_2 end_pose_theta_2];
            [clotoid_check_2, f_p_2, L ] = clotoide(end_pose_2, start_pose_2, numberOfIntermediatePose, min_radius);

            p = flip(f_p);
            p_2= flip(f_p_2);
            
            if (clotoid_check == true && clotoid_check_2 == true)
                [primID, send_primitives] = primitive_function(start_pose, end_pose, p, start_pose_2, end_pose_2, p_2, primID, resolution, curve_bacward_penalty, curve_bacward_penalty, angles, print_stuff);
                primitives(primID-2) = send_primitives(primID-2);
                primitives(primID-1) = send_primitives(primID-1);
                break
            end 
        end
        straight_start_pose_theta = angles(degree_number);
        straight_start_pose = [start_position straight_start_pose_theta];

        straight_end_pose_long = [end_pose(1) end_pose(2) angles(degree_number)];
        [clotoid_check_straight_long, f_p_straight_long, L ] = clotoide(straight_end_pose_long, straight_start_pose, numberOfIntermediatePose, min_radius);
        
        p_straight_long = flip(f_p_straight_long);
        
        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose_long, p_straight_long, primID, resolution, long_bacward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1);
        end
        
        %  d5 and d6
        if(degree_number == 12)
        for x_position = for_loop_search
            final_position = [x_position x_position*2];
            end_pose = [final_position end_pose_theta];
            [clotoid_check, f_p, L ] = clotoide(end_pose, start_pose, numberOfIntermediatePose, min_radius);

            x_position_2 = x_position;
            final_position_2 = [x_position_2 x_position_2*2];
            end_pose_2 = [final_position_2 end_pose_theta_2];
            [clotoid_check_2, f_p_2, L ] = clotoide(end_pose_2, start_pose_2, numberOfIntermediatePose, min_radius);

            p = flip(f_p);
            p_2= flip(f_p_2);
            
            if (clotoid_check == true && clotoid_check_2 == true)
                [primID, send_primitives] = primitive_function(start_pose, end_pose, p, start_pose_2, end_pose_2, p_2, primID, resolution, curve_bacward_penalty, curve_bacward_penalty, angles, print_stuff);
                primitives(primID-2) = send_primitives(primID-2);
                primitives(primID-1) = send_primitives(primID-1);
                break
            end 
        end
        straight_start_pose_theta = angles(degree_number);
        straight_start_pose = [start_position straight_start_pose_theta];

        straight_end_pose_long = [end_pose(1) end_pose(2) angles(degree_number)];
        [clotoid_check_straight_long, f_p_straight_long, L ] = clotoide(straight_end_pose_long, straight_start_pose, numberOfIntermediatePose, min_radius);
        
        p_straight_long = flip(f_p_straight_long);
        
        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose_long, p_straight_long, primID, resolution, long_bacward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1);
        end
     end
    
    
    
% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    if (degree_number == 13 || degree_number == 14 || degree_number == 15 || degree_number == 16) %% Quadrant 4
        if ( degree_number ~= 16)
        start_pose_theta = angles(degree_number+1);
        start_pose = [start_position start_pose_theta];

        end_pose_theta = angles(degree_number-1);

        start_pose_theta_2 = angles(degree_number-1);    
        start_pose_2 = [start_position start_pose_theta_2];

        end_pose_theta_2 = angles(degree_number+1);
        end
        % d1
        if (degree_number == 13)
        for y_position = for_loop_search
            final_position = [0 y_position];
            end_pose = [final_position end_pose_theta];
            [clotoid_check, f_p, L ] = clotoide(end_pose, start_pose, numberOfIntermediatePose, min_radius);

            y_position_2 = y_position;
            final_position_2 = [0 y_position_2];
            end_pose_2 = [final_position_2 end_pose_theta_2];
            [clotoid_check_2, f_p_2, L ] = clotoide(end_pose_2, start_pose_2, numberOfIntermediatePose, min_radius);

            p = flip(f_p);
            p_2= flip(f_p_2);
            
            if (clotoid_check == true && clotoid_check_2 == true)
                [primID, send_primitives] = primitive_function(start_pose, end_pose, p, start_pose_2, end_pose_2, p_2, primID, resolution, curve_bacward_penalty, curve_bacward_penalty, angles, print_stuff);
                primitives(primID-2) = send_primitives(primID-2);
                primitives(primID-1) = send_primitives(primID-1);
                break 
            end   
        end
        straight_start_pose_theta = angles(degree_number);
        straight_start_pose = [start_position straight_start_pose_theta];

        straight_end_pose_long = [end_pose(1) end_pose(2) angles(degree_number)];
        [clotoid_check_straight_long, f_p_straight_long, L ] = clotoide(straight_end_pose_long, straight_start_pose, numberOfIntermediatePose, min_radius);
        
        p_straight_long = flip(f_p_straight_long);
        
        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose_long, p_straight_long, primID, resolution, long_bacward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1);
        end

        % d2 and d3
        if(degree_number == 14)
        for x_position = for_loop_search
            final_position = [-x_position x_position*2];
            end_pose = [final_position end_pose_theta];
            [clotoid_check, f_p, L ] = clotoide(end_pose, start_pose, numberOfIntermediatePose, min_radius);

            x_position_2 = x_position;
            final_position_2 = [-x_position_2 x_position_2*2];
            end_pose_2 = [final_position_2 end_pose_theta_2];
            [clotoid_check_2, f_p_2, L ] = clotoide(end_pose_2, start_pose_2, numberOfIntermediatePose, min_radius);

            p = flip(f_p);
            p_2= flip(f_p_2);
            
            if (clotoid_check == true && clotoid_check_2 == true)
                [primID, send_primitives] = primitive_function(start_pose, end_pose, p, start_pose_2, end_pose_2, p_2, primID, resolution, curve_bacward_penalty, curve_bacward_penalty, angles, print_stuff);
                primitives(primID-2) = send_primitives(primID-2);
                primitives(primID-1) = send_primitives(primID-1);
                break
            end 
        end  
        straight_start_pose_theta = angles(degree_number);
        straight_start_pose = [start_position straight_start_pose_theta];

        straight_end_pose_long = [end_pose(1) end_pose(2) angles(degree_number)];
        [clotoid_check_straight_long, f_p_straight_long, L ] = clotoide(straight_end_pose_long, straight_start_pose, numberOfIntermediatePose, min_radius);
        
        p_straight_long = flip(f_p_straight_long);
        
        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose_long, p_straight_long, primID, resolution, long_bacward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1);
        end

        % d4 
        if(degree_number == 15)
        for x_position = for_loop_search
            y_position = x_position;
            final_position = [-x_position y_position];
            end_pose = [final_position end_pose_theta];
            [clotoid_check, f_p, L ] = clotoide(end_pose, start_pose, numberOfIntermediatePose, min_radius);

            x_position_2 = x_position;
            y_position_2 = x_position_2;
            final_position_2 = [-x_position_2 y_position_2];
            end_pose_2 = [final_position_2 end_pose_theta_2];
            [clotoid_check_2, f_p_2, L ] = clotoide(end_pose_2, start_pose_2, numberOfIntermediatePose, min_radius);

            p = flip(f_p);
            p_2= flip(f_p_2);
            
            if (clotoid_check == true && clotoid_check_2 == true)
                [primID, send_primitives] = primitive_function(start_pose, end_pose, p, start_pose_2, end_pose_2, p_2, primID, resolution, curve_bacward_penalty, curve_bacward_penalty, angles, print_stuff);
                primitives(primID-2) = send_primitives(primID-2);
                primitives(primID-1) = send_primitives(primID-1);
                break
            end 
        end
        straight_start_pose_theta = angles(degree_number);
        straight_start_pose = [start_position straight_start_pose_theta];

        straight_end_pose_long = [end_pose(1) end_pose(2) angles(degree_number)];
        [clotoid_check_straight_long, f_p_straight_long, L ] = clotoide(straight_end_pose_long, straight_start_pose, numberOfIntermediatePose, min_radius);
        
        p_straight_long = flip(f_p_straight_long);
        
        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose_long, p_straight_long, primID, resolution, long_bacward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1);
        end
       
        
        % d5 and d6
        if(degree_number == 16)
            start_pose_theta = angles(1);
            start_pose = [start_position start_pose_theta];

            end_pose_theta = angles(degree_number-1);

            start_pose_theta_2 = angles(degree_number-1);    
            start_pose_2 = [start_position start_pose_theta_2];

            end_pose_theta_2 = angles(1);
            
        for y_position = for_loop_search
            final_position = [-y_position*2 y_position];
            end_pose = [final_position end_pose_theta];
            [clotoid_check, f_p, L ] = clotoide(end_pose, start_pose, numberOfIntermediatePose, min_radius);

            y_position_2 = y_position;
            final_position_2 = [-y_position_2*2 y_position_2];
            end_pose_2 = [final_position_2 end_pose_theta_2];
            [clotoid_check_2, f_p_2, L ] = clotoide(end_pose_2, start_pose_2, numberOfIntermediatePose, min_radius);

            p = flip(f_p);
            p_2= flip(f_p_2);
            
            if (clotoid_check == true && clotoid_check_2 == true)
                [primID, send_primitives] = primitive_function(start_pose, end_pose, p, start_pose_2, end_pose_2, p_2, primID, resolution, curve_bacward_penalty, curve_bacward_penalty, angles, print_stuff);
                primitives(primID-2) = send_primitives(primID-2);
                primitives(primID-1) = send_primitives(primID-1);
                break
            end 
        end
        straight_start_pose_theta = angles(degree_number);
        straight_start_pose = [start_position straight_start_pose_theta];

        straight_end_pose_long = [end_pose(1) end_pose(2) angles(degree_number)];
        [clotoid_check_straight_long, f_p_straight_long, L ] = clotoide(straight_end_pose_long, straight_start_pose, numberOfIntermediatePose, min_radius);
        
        p_straight_long = flip(f_p_straight_long);
        
        [primID, send_primitives] = primitive_function_2(straight_start_pose, straight_end_pose_long, p_straight_long, primID, resolution, long_bacward_penalty, angles, print_stuff);
        primitives(primID-1) = send_primitives(primID-1);
        end    
    end
    
end

end  %% bacward move end-----------------------------------------------------------------------------------

%% generate file

set(gca,'DataAspectRatio',[1 1 1]);

generate_file(primitives, numberOfAngle, resolution);


%% primitive collection and plot function

function [primID, send_primitives] = primitive_function(start_pose, end_pose, p, start_pose_2, end_pose_2, p_2, primID, resolution, forward_penalty, forward_penalty_2, angles, print_stuff)
    send_primitives(primID).real_start_pose = start_pose;
    send_primitives(primID).real_end_pose = end_pose;
    send_primitives(primID).intermediate_poses = p;
    send_primitives(primID).additionalActionCostMult = forward_penalty;

    send_primitives(primID).start_orientation = find(angles==start_pose(3))-1;
    prim_end_pose = [end_pose(1)/resolution end_pose(2)/resolution find(angles==end_pose(3))-1];
    send_primitives(primID).end_pose = prim_end_pose;
    primID = primID + 1;

    send_primitives(primID).real_start_pose = start_pose_2;
    send_primitives(primID).real_end_pose = end_pose_2;
    send_primitives(primID).intermediate_poses = p_2;
    send_primitives(primID).additionalActionCostMult = forward_penalty_2;
    send_primitives(primID).start_orientation = find(angles==start_pose_2(3))-1;
    prim_end_pose = [end_pose_2(1)/resolution end_pose_2(2)/resolution find(angles==end_pose_2(3))-1];
    send_primitives(primID).end_pose = prim_end_pose;
    primID = primID + 1;
    
    if(print_stuff==1)
        plot(send_primitives(primID-1).intermediate_poses(:,1),send_primitives(primID-1).intermediate_poses(:,2),'+-b');
        plot(send_primitives(primID-2).intermediate_poses(:,1),send_primitives(primID-2).intermediate_poses(:,2),'+-b');
        grid on;
        hold on;
        quiver(send_primitives(primID-1).intermediate_poses(:,1),send_primitives(primID-1).intermediate_poses(:,2),...
            cos(send_primitives(primID-1).intermediate_poses(:,3)),sin(send_primitives(primID-1).intermediate_poses(:,3)),0.1,...
            'Color',[0 0 0]);
        quiver(send_primitives(primID-2).intermediate_poses(:,1),send_primitives(primID-2).intermediate_poses(:,2),...
            cos(send_primitives(primID-2).intermediate_poses(:,3)),sin(send_primitives(primID-2).intermediate_poses(:,3)),0.1,...
            'Color',[0 0 0]);
        pause(0.1);
    end
end


function [primID, send_primitives] = primitive_function_2(start_pose, end_pose, p, primID, resolution, penalty, angles, print_stuff)
    send_primitives(primID).real_start_pose = start_pose;
    send_primitives(primID).real_end_pose = end_pose;
    send_primitives(primID).intermediate_poses = p;
    send_primitives(primID).additionalActionCostMult = penalty;

    send_primitives(primID).start_orientation = find(angles==start_pose(3))-1;
    prim_end_pose = [end_pose(1)/resolution end_pose(2)/resolution find(angles==end_pose(3))-1];
    send_primitives(primID).end_pose = prim_end_pose;
    primID = primID + 1;

    if(print_stuff==1)
        plot(send_primitives(primID-1).intermediate_poses(:,1),send_primitives(primID-1).intermediate_poses(:,2),'+-b');
        grid on;
        hold on;
        quiver(send_primitives(primID-1).intermediate_poses(:,1),send_primitives(primID-1).intermediate_poses(:,2),...
            cos(send_primitives(primID-1).intermediate_poses(:,3)),sin(send_primitives(primID-1).intermediate_poses(:,3)),0.1,...
            'Color',[0 0 0]);
        pause(0.1);
    end
end




