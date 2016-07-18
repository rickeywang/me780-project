function [weighting] = apply_weighting(minDist)
% Find maximum distance, apply weighting by using 1 - dist/max_dist

% Find maximum distance:
maxDist = minDist(1);
for i=2:length(minDist)
    if minDist(i) > maxDist
        maxDist = minDist(i);
    end
end

% Compute weighting for each point
weighting = zeros(length(minDist),1);

for i=1:length(minDist)
    weighting(i) = 1-minDist(i)/maxDist;
end

% Normalize the weightings
weighting_sum = sum(weighting);

for i=1:length(minDist)
    weighting(i) = weighting(i)/weighting_sum;
end