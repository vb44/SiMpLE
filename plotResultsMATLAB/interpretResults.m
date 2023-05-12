% -------------------------------------------------------------------------
% Example of interepreting the result file.
% -------------------------------------------------------------------------
clear;
clc;
close all;

% convert the results file to (6x1) vector pose estimates
filePath = "~/Documents/public_repositories/SiMpLE/results/test_KITTI_10";
res = readmatrix(filePath);

poseEstimates = zeros(length(res),6);
for i = 1:length(res)
    poseEstimates(i,:) = hom2rpyxyz([res(i,1:4);
                                     res(i,5:8);
                                     res(i,9:12);
                                     0 0 0 1]);
end

% plot the result
clf;
hold on;
box on;
axis equal;
grid on;
plot3(poseEstimates(:,4),poseEstimates(:,5),poseEstimates(:,6),'.b')
labelFontSize = 20;
xlabel('$X $ [m]','Interpreter','latex','FontSize',labelFontSize);
ylabel('$Y $ [m]','Interpreter','latex','FontSize',labelFontSize);
zlabel('$Z $ [m]','Interpreter','latex','FontSize',labelFontSize);
set(gca,'TickLabelInterpreter','latex','FontSize',labelFontSize);
view(45,20)

% -------------------------------------------------------------------------
% Helper functions.
% -------------------------------------------------------------------------
function rpyxyz = hom2rpyxyz(T)
% helper function for converting a (4x4) homogenous matrix to 
% a 6x1 (roll[rad],pitch[rad],yaw[rad],x[m],y[m],z[m]) vector.
    rot = T(1:3,1:3);
    roll = atan2(rot(3,2),rot(3,3));
    pitch = asin(-rot(3,1));
    yaw = atan2(rot(2,1),rot(1,1));
    x = T(1,4);
    y = T(2,4);
    z = T(3,4);
    rpyxyz = [roll, pitch, yaw, x, y, z];
end