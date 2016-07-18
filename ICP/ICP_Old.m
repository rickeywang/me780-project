ptCloudFull = pcread('teapot.ply');

fullPCMatrix = ptCloudFull.Location(:,:);

ptCloud = pcdownsample(ptCloudFull,'random',0.05);

Np = ptCloud.Count;

% Apply transform
A = [cos(pi/6) sin(pi/6) 0 0; ...
    -sin(pi/6) cos(pi/6) 0 0; ...
            0         0  1 0; ...
            5         5  0 1];
        

tform1 = affine3d(A);

ptCloudTformed = pctransform(ptCloud,tform1);

figure(1); clf; hold on;
showPointCloud(ptCloudTformed);

iterations = 30;

totalT = zeros(3, 1, iterations);
totalR = zeros(3, 3, iterations);

for i=1:iterations
    totalR(:, :, i) =  eye(3);
end

p = ptCloud.Location(:,:)';
pt = p;
q = ptCloudTformed.Location(:,:)';


%% ICP - Algorithm
for cco=1:iterations
    [point minDist] = min_dist_points(pt, q);
    %% Compute weighting for each point
    %weighting = apply_weighting(minDist);
    weighting = zeros(1,Np) + 1;
    weighting = weighting./Np;
    %% Eliminate outliers
    
    
    %% Compute centroids of point clouds
    centroidRef = [0; 0; 0];
    centroidTformed = [0; 0; 0];
    
    for i=1:ptCloud.Count
        centroidRef(1) = centroidRef(1) + pt(1,i);
        centroidRef(2) = centroidRef(2) + pt(2,i);
        centroidRef(3) = centroidRef(3) + pt(3,i);
    end
    centroidRef = centroidRef / Np;
    
    for i=1:ptCloudTformed.Count
        centroidTformed(1) = centroidTformed(1) + q(1,i)*weighting(i);
        centroidTformed(2) = centroidTformed(2) + q(2,i)*weighting(i);
        centroidTformed(3) = centroidTformed(3) + q(3,i)*weighting(i);
    end
    %centroidTformed = centroidTformed / ptCloudTformed.Count;
    
    
    
    %% Find translation between the 2 centroids
    pt_centred = pt - repmat(centroidRef,1,length(pt));
    q_centred = q - repmat(centroidTformed, 1, length(q));
    
    weight_matrix = diag(weighting);
    
    %% Find the optimal rotation matrix between the 2 point clouds using SVD
    N = pt_centred * weight_matrix * q_centred';
    [U, ~ , V] = svd(N);
    R = V*diag([1 1 det(U*V')])*transpose(U);
    T = centroidTformed - R*centroidRef;
    
    zRot = atan2(R(2,1),R(1,1));
    
    totalR(:, :, cco+1) = R*totalR(:, :, cco);
    totalT(:, :, cco+1) = R*totalT(:, :, cco) + T;
    
    % Apply transformation
    pt(:, :) = totalR(:, :, cco+1)*p(:,:) + repmat(totalT(:,:,cco+1),1,Np);
end



    