function output = world_data(setup_img)
%[] Creating masks for obstacles, initial position, goal position

% Mask for obstacle
newim_obstacle_bin = createMaskObstacle_red(setup_img);
newim_obstacle_bin = bwareaopen(newim_obstacle_bin,250);
newim_obstacle_bin = imfill(newim_obstacle_bin, 'holes');

% Mask for robot initial position
newim_robo_init_position = createMaskhdblue(setup_img);
newim_robo_init_position = bwareaopen(newim_robo_init_position,45);
newim_robo_init_position = imfill(newim_robo_init_position, 'holes');

% Mask for robot goal position
newim_goal_position = createMaskGoal(setup_img);
newim_goal_position = bwareaopen(newim_goal_position,90);
newim_goal_position = imfill(newim_goal_position, 'holes');

% Calculating the bounding box of obstacle region 

[labeledImage, numberOfRegions] = bwlabel(newim_obstacle_bin);

count = 0;

stats = regionprops(labeledImage, 'BoundingBox');
for rb = 1:numberOfRegions
    count = count + 1;
    BB(count,:) = stats(rb).BoundingBox;
end

% Calculating initial position of robot

[labeledImage, numberOfRegions] = bwlabel(newim_robo_init_position);

count = 0;
cent = zeros(numberOfRegions,2);

stats = regionprops(labeledImage,'Centroid');
for rb = 1:numberOfRegions
    count = count + 1;
    cent(count,:) = stats(rb).Centroid;
    cent(count,2) = 1080 - cent(count,2) ;  % Correction for y-axis.
end

robo_init_position = cent;

% Calculating the goal/target position for robot

[labeledImage, numberOfRegions] = bwlabel(newim_goal_position);

count = 0;
cent = zeros(numberOfRegions,2);

stats = regionprops(labeledImage,'Centroid');
for rb = 1:numberOfRegions
    count = count + 1;
    cent(count,:) = stats(rb).Centroid;
    cent(count,2) = 1080 - cent(count,2) ;  % Correction for y-axis.
end

target_position = cent;

% Outputs
output.obstacle_bin_img = newim_obstacle_bin;
output.robo_init_position = robo_init_position;
output.target_position = target_position;
output.obstaclesBB = BB;

% Plot
figure(1)
imshow(setup_img)
hold on
% Bounding box
for ii = 1:size(BB,1)
rectangle('Position',BB(ii,:),'EdgeColor','y','LineWidth',3);
end
% Target/Goal position
plot(target_position(:,1),1080-target_position(:,2),'r*','LineWidth',0.7,'MarkerSize',5)
viscircles([target_position(:,1) 1080-target_position(:,2)],19.4377*8.5254,'LineStyle','--');
% Initial position
plot(robo_init_position(:,1),1080-robo_init_position(:,2),'*','Color',[51 255 51]/255,...
    'LineWidth',0.7,'MarkerSize',5)
end
