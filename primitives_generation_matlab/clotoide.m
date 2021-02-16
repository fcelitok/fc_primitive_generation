%function [p L] = clotoide(p1,p2,N)
%
%Computes a clothoid funtion between two points and provides its
%interpolation
%
% Input:
%   - p1: [x1 y1 theta1]
%   - p2: [x2 y2 theta2]
%   - N: number of samples in the trajectory
%
% Output
%   - p: Nx3 vector of points in the trajectory in the form x, y, theta
%   - L: lenght of the trajectory 
%
% Example
%  [p L]=clotoide([0 0 0], [4 0.5 pi/8],20);
%  plot(p(:,1),p(:,2),'+-b'), grid, axis equal;
%  hold on, quiver(p(:,1),p(:,2),cos(p(:,3)),sin(p(:,3)),0.1,'Color',[0 0 0])

function [check p L] = clotoide(p1,p2,N,minimum_radius)
check = true;
addpath('ebertolazzi-G1fitting-a338d59/G1fitting')

[k,dk,L,iter] = buildClothoid(p1(1),p1(2),p1(3),p2(1),p2(2),p2(3));
[X Y] = pointsOnClothoid( p1(1), p1(2), p1(3), k, dk, L, N);

% points = [0 L/2];
% 
% for t = 1:length(points)   
%     [Xvalue,Yvalue,TH,curvature] = evalClothoid( p1(1), p1(2), p1(3), k, dk, points(t));
%     if (abs(1/curvature) < minimum_radius)
%         check = false;
%     end
% end



for t = 0:N-1   % 0:N-1 original  % 0:N-4 reduced 
    [Xvalue,Yvalue,TH,curvature] = evalClothoid( p1(1), p1(2), p1(3), k, dk, (t*L/(N-1)));
    if (abs(1/curvature) < minimum_radius)
        check = false;
    end
end

%%%%%%%%%%%%%%%% turning radius check %%%%%%%%%%%%%%%%
% 
% initial_turning_radius = abs(1/k);
% final_turning_radius = abs(1/(dk*L+k));
% min_radius = 0.2;  % 0.2 for 0.2 meter vehicle
% if(initial_turning_radius < min_radius)        % turning radius check you must change min radius
%     check = false;
% elseif(final_turning_radius < min_radius)
%     check = false;
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%[XX2,YY2,TH2,KK2] = evalClothoid( p1(1), p1(2), p1(3), k, dk, 0) ;
% initial = [XX2,YY2,TH2,KK2]
% initial_curvature = k
% [XXh,YYh,THh,KKh] = evalClothoid( p1(1), p1(2), p1(3), k, dk, L/2);
% half = [XXh,YYh,THh,KKh]
% [XX,YY,TH,KK] = evalClothoid( p1(1), p1(2), p1(3), k, dk, L) ;
% final = [XX,YY,TH,KK]
% final_curvature = dk*L+k
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

len = 0;
theta(1)=p1(3);
for i=2:N
    theta(i) = atan2(Y(i)-Y(i-1), X(i)-X(i-1));
end

p=[X(1) Y(1) p1(3)];
for i=2:N-1
    p=[p ; X(i) Y(i) atan2((sin(theta(i+1))+sin(theta(i)))/2, (cos(theta(i+1))+cos(theta(i)))/2)];
end
p=[p ; X(N) Y(N) p2(3)];






