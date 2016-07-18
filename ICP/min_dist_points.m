function [point minDist] = min_dist_points(reference, transformed)
    distance = 0;
    minimum = 0;
    point = zeros(1,length(transformed));
    minDist = zeros(size(point));
    
    
    % Find the nearest point from the transformed point cloud to the
    % reference cloud

    for i=1:length(transformed)
        for j=1:length(reference)
            X = reference(1,j) - transformed(1,i);
            Y = reference(2,j) - transformed(2,i);
            Z = reference(3,j) - transformed(3,i);
            distance = sqrt(X*X + Y*Y + Z*Z);
            
            % If this is the first comparison, take it as our new minimum
            % distance
            if (j==1)
                minimum = distance;
                point(i) = j;
            end
            
            % If the new distance is less than the current minimum, update the
            % minimum, and save the point to which it corresponds
            if (distance < minimum)
                minimum = distance;
                point(i) = j;
            end
        end
        
        % Save the minimum distance as well as the corresponding point
        minDist(i) = minimum;
    end
end