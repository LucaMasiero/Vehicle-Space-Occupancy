function plotResults_p1(worldPoints, R, parallelepiped)
% PLOTRESULTS_P1 - Plot the feature points localized in the world
% reference system and, if required, the car represented as a
% parallelepiped.
%
% Inputs:  
%   worldPoints - [3,4] matrix containing [x;y;z] coordinates of the
%                 feature points in the world reference system
%   R - camera rotation matrix
%   parallelepiped - boolean variable specifying if the parallelepiped has
%                    to be drawn (true) or not (false)
%
    
    figure()
    hold on
    title("Camera and Car as parallelepiped in World reference frame")
    
    % Plot camera reference system
    X = R(:,1);
    Y = R(:,2);
    Z = R(:,3);

    qx = quiver3(0,0,0, X(1),X(2),X(3),100,'r');
    qy = quiver3(0,0,0, Y(1),Y(2),Y(3),100,'g');
    qz = quiver3(0,0,0, Z(1),Z(2),Z(3),100,'b');
    
    % Plot quadrangle
    x = worldPoints(1,:);
    y = worldPoints(2,:);
    z = worldPoints(3,:);
    scatter3(x,y,z, 'bo');
    fill3(x,y,z,'b');
    
    % Plot parallelepiped
    if parallelepiped
        [xPatch, yPatch, zPatch] = defineCarPatches(worldPoints(:,4));
        patch(xPatch,yPatch,zPatch,'c','FaceAlpha',.5);
    end
    
    % Plot camera
    scatter3(0,0,0, 'r+');  % camera center
    pose = rigid3d(R',[0,0,0]);
    plotCamera("AbsolutePose",pose,"Size",15)
    
    hold off
    legend([qx,qy,qz], 'X_{cam}', 'Y_{cam}', 'Z_{cam}')
    
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    grid on
    %axis equal
    
    xlim([-300,300])
    
    axis equal
    view(-30,30)
    % Turn axis as world reference system
    ax = gca;
    ax.XDir = "reverse";
    ax.ZDir = "reverse";
end

function [xPatch, yPatch, zPatch] = defineCarPatches(BL)
% DEFINECARPATCHES - Defines the patches needed to plot the parallelepiped.
%
% Inputs:
%   BL - Bottom Left corner of the license plate, starting from which the
%   parallelepiped will be defined
%
    % Parallelepiped vertices
    width = 157.8;
    height = 157.8;
    length = 353.8;
    offset_width = 52.5;
    offset_length = -15;
    offset_height = 75;
    
    P1 = BL + [offset_width,offset_length,offset_height]';
    P2 = P1 + [-width,0,0]';
    P3 = P1 + [0,0,-height]';
    P4 = P2 + [0,0,-height]';
    P5 = P1 + [0,length,0]';
    P6 = P2 + [0,length,0]';
    P7 = P3 + [0,length,0]';
    P8 = P4 + [0,length,0]';
    
    back_side = [P1,P2,P4,P3]';
    front_side = [P5,P6,P8,P7]';
    left_side = [P1,P3,P7,P5]';
    right_side = [P2,P6,P8,P4]';
    top_side = [P3,P4,P8,P7]';
    bottom_side = [P1,P2,P6,P5]';
    xPatch= [back_side(:,1), front_side(:,1), left_side(:,1), right_side(:,1), top_side(:,1), bottom_side(:,1)];
    yPatch = [back_side(:,2), front_side(:,2), left_side(:,2), right_side(:,2), top_side(:,2), bottom_side(:,2)];
    zPatch = [back_side(:,3), front_side(:,3), left_side(:,3), right_side(:,3), top_side(:,3), bottom_side(:,3)];
end