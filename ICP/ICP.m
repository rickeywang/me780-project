clear all;
ptCloudFull = pcread('teapot.ply');

ptCloudRef = ptCloudFull;
ptCloudNew = ptCloudFull;

fullPCMatrix = ptCloudFull.Location(:,:);

refcc = 1;
newcc = 1;
thresh = 1.5;
for i=1:ptCloudFull.Count
    if fullPCMatrix(i,2) < thresh 
        ref(refcc,:) = fullPCMatrix(i,:);
        refcc = refcc+1;
    end
    if ptCloudFull.Location(i,2) > -thresh 
        new(newcc,:) = fullPCMatrix(i,:); 
        newcc = newcc+1;
    end
end

ptCloudRef = pointCloud(ref);
ptCloudNew = pointCloud(new);

figure(1); clf; hold on;
showPointCloud(ptCloudRef);
 
figure(2); clf; hold on;
showPointCloud(ptCloudNew);

Np = ptCloudRef.Count;

% Apply transform
A = [cos(pi/6) sin(pi/6) 0 0; ...
    -sin(pi/6) cos(pi/6) 0 0; ...
            0         0  1 0; ...
            5         5  0 1];
        

tform1 = affine3d(A);

ptCloudNew = pctransform(ptCloudNew,tform1);

% figure(1); clf; hold on;
% showPointCloud(ptCloudRef);
% 
% figure(2); clf; hold on;
% showPointCloud(ptCloudNew);

iterations = 30;

totalT = zeros(3, 1, iterations);
totalR = zeros(3, 3, iterations);

for i=1:iterations
    totalR(:, :, i) =  eye(3);
end

p = ptCloudRef.Location(:,:)';
q = ptCloudNew.Location(:,:)';

% ptCloudRef = pcdownsample(ptCloudRef,'random',0.05);
% ptCloudNew = pcdownsample(ptCloudNew,'random',0.05);

pt = p;
%pds = ptCloudRef.Location(:,:)';
%qds = ptCloudNew.Location(:,:)';


%% ICP - Algorithm
for cco=1:iterations
    transformedPC = pointCloud(pt');
    transformedDS = pcdownsample(transformedPC,'random', 0.05);
    ptds = transformedDS.Location(:,:)';
    
    newPC = pcdownsample(ptCloudNew,'random',0.05);
    qds = newPC.Location(:,:)';
    
    Np = length(ptds);
    
    [point minDist] = min_dist_points(ptds, qds);
    %% Compute weighting for each point
    %weighting = apply_weighting(minDist);
    weighting = zeros(1,Np) + 1;
    weighting = weighting./Np;
    %% Eliminate outliers
    
    
    %% Compute centroids of point clouds
    centroidRef = [0; 0; 0];
    centroidTformed = [0; 0; 0];
    
    for i=1:Np;
        centroidRef(1) = centroidRef(1) + ptds(1,i);
        centroidRef(2) = centroidRef(2) + ptds(2,i);
        centroidRef(3) = centroidRef(3) + ptds(3,i);
    end
    centroidRef = centroidRef / Np;
    
    for i=1:Np;
        centroidTformed(1) = centroidTformed(1) + qds(1,i)*weighting(i);
        centroidTformed(2) = centroidTformed(2) + qds(2,i)*weighting(i);
        centroidTformed(3) = centroidTformed(3) + qds(3,i)*weighting(i);
    end
    %centroidTformed = centroidTformed / ptCloudTformed.Count;
    
    
    
    %% Find translation between the 2 centroids
    pt_centred = ptds - repmat(centroidRef,1,length(ptds));
    q_centred = qds - repmat(centroidTformed, 1, length(qds));
    
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
    pt(:, :) = totalR(:, :, cco+1)*p(:,:) + repmat(totalT(:,:,cco+1),1,length(p));
end



    