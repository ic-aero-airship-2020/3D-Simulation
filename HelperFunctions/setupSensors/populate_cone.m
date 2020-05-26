%function that populates the conic sections with interior data points 
%-----------------------------------------------------------------------------------------------------------------------
% inputs - 
    % - X =  x data points that currently define the conic mesh (x_cone)
    % - Y =  y data points that currently deffine the conic mesh (rangeToF)
    % - Z =  z data points that currently define the conic mesh of (z_cone)
    % - no_sens = nnumber of sensors
    % -  resolution = how many piece wise discretisations of the local radius you want to have a resolution to
           % - e.g if resolution = 100 (100 disccretisations in the radial
           % direction
    % - ang_set = set of angles for angular rotation about z-axis
    % - ang_inc = inclination angle of the sensors from the horizontal plane
    % - position = Position vector of the sensors 
    
    
% outputs - 
    % - out = a structure data type that contains the populated x y z data
              % points of the conic mesh for each individial sensor
              
              %Note: top row in (X,Y,Z)data field denotes the centre of the
              %cone (i.e local radius is zero); the bottom row represents
              %the coordinate position when the local radius = height of
              %the cone at that planar cross section
              
%----------------------------------------------------------------------------------------------------------------------

function out = populate_cone(X,Y,Z,no_sens,resolution,ang_set,ang_inc,position)
out = struct();
% find matrix size and number of sensors (no_sens)
%conic length = no. discretisations in long axis of the cone
%conic_radial = no. discretisations radialy at a give length position along the one 
[conic_length,conic_radial] = size(X);
omega = [0:2*pi/(conic_radial - 1):2*pi];
%make sure position vector format is a 3 x n block
pos_size = size(position);
if (length(pos_size)==3)
    for i = 1:pos_size(3);
    Place(:,i) = position(:,1,i); 
    end
else
    Place = position;
end
%initialise data structure
for i = 1:no_sens
    out.(sprintf('Sensor%d',i)).Xdata = zeros(resolution,conic_radial,conic_length);
    out.(sprintf('Sensor%d',i)).Ydata = zeros(resolution,conic_radial,conic_length);
    out.(sprintf('Sensor%d',i)).Zdata = zeros(resolution,conic_radial,conic_length);
end

%calculate the radius at every point along the conic_length
for i =1:no_sens
for j = 1:conic_length
    
    radius = sqrt((X(j,1)^2)+(Z(j,1)^2));
    local_radius = [0:radius/(resolution-1):radius]';
    if (radius == 0)
        local_radius = zeros(resolution,1);
    end
    x = local_radius.*cos(omega);
    y = Y(j,1)*ones(resolution,conic_radial);
    z = local_radius.*sin(omega);
    
    for k = 1:resolution
    %define plane matrix of disk
    MAT = [x(k,:);y(k,:);z(k,:)];
    % rotate and translate the plane matrix to the desired location;
    NMAT = rotz(ang_set(i))*rotx(ang_inc)*MAT + Place(:,i);
    
    out.(sprintf('Sensor%d',i)).Xdata(k,:,j) = [NMAT(1,:)] ;
    out.(sprintf('Sensor%d',i)).Ydata(k,:,j) = [NMAT(2,:)] ;
    out.(sprintf('Sensor%d',i)).Zdata(k,:,j) = [NMAT(3,:)] ;
    end
    
    
    
end
end
end