function [new_position,current_orientation,orientation_theta] = FK_simulation_step(dp,dp_theta,sequence_length,current_position,current_orientation)
 
theta = current_orientation;

SE2 = @(theta) [cos(theta),-sin(theta); sin(theta),cos(theta)];

for i = 1:sequence_length
    new_position(:,i) = (SE2(theta)*dp)+current_position;
    current_position = new_position(:,i);
    theta = (current_orientation+dp_theta);
    current_orientation = theta;
    orientation_theta(i,:) = theta;
end
end
