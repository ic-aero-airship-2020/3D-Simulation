function sensorLines = ConfigureSensors(airship)
    % Setting up single sensor config
    sensorParams = airship.sensorSetup;
    Sensor = setupGeneralSensor(sensorParams.dayRange, sensorParams.nightRange, sensorParams.FoV, sensorParams.sensorSize);
    
    balloonParams = airship.balloonSetup;
    radius = balloonParams.radius;
    theta  = -pi/2 :pi/(180) : pi/2;

    SenLines = balloonParams.numberOfSensors; % equal to the number of sensors
    phi = 360/(SenLines); % rotation angle between each senssor plane
    ang_set = [phi:phi:360]; % set of angles for rotation plaes
    
    
    % define initial semi circle in the z - y plane for baloon shape
    Z = radius*sin(theta);
    Y = radius*cos(theta);
    X = zeros(1,length(theta));

    MAT = [X;Y;Z]; % shape matrix data for baloon
    
    % define shape matrix for sensor line
    for i = 1:SenLines
        NMAT(:,:,i) = rotz(ang_set(i))*MAT;
    end

    % define sensor shape array in x-z plane centred at origin in y- direction 
    SMAT = [Sensor.ToFSensor.Shape(1,:);zeros(1,length(Sensor.ToFSensor.Shape(1,:)));Sensor.ToFSensor.Shape(2,:)];
    %Find coordinate on the sensor line to place sensor
    ang_Place = sensorParams.refSensorHeading;  %between -90 and - 90 degrees
    idx = 91 + ang_Place;
    for i = 1:SenLines
        SPlace(:,:,i) = NMAT(:,idx,i);
        %transalte and rotate sensor shape matrix to sensor location
        SNMAT(:,:,i) = rotz(ang_set(i))*rotx(ang_Place)*SMAT + SPlace(:,:,i);

    end
    
    % define conic Fov for sensor; cone growing with y-axis as the rotation
    % axis
    omega    = [0:2*pi/59:2*pi];
    rangeToF = Sensor.ToFSensor.Range.day;     %change to night if needed
%     rangeToF = [0:rangeToF/179:rangeToF];
    rangeToF = [0,rangeToF];
    rad_cone = tand(Sensor.ToFSensor.FoV/2)*rangeToF;
    x_cone   = rad_cone'.*cos(omega);
    z_cone   = rad_cone'.*sin(omega);
%     rangeToF = rangeToF'.*ones(180,60);
    rangeToF = rangeToF'.*ones(2,60);
    
    figure('Name','Sensor FoV visualisation');
    mesh(x_cone,rangeToF,z_cone);
    title('Sensor conic field of vision visualised');
%     
%     sensorLines.x = x_cone;
%     sensorLines.y = rangeToF;
%     sensorLines.z = z_cone;
    
    Cone_angle_place = balloonParams.sensorZPlaneRot; %angle of inclination from the horizontal pplane from the sensor


    for j = 1:size(rangeToF,1)
        %transalte and rotate cone matrix to sensor location
        CMAT=rotx(Cone_angle_place)*[x_cone(j,:);rangeToF(j,:);z_cone(j,:)];
        Sx_cone(j,:) = CMAT(1,:);
        SrangeToF(j,:)= CMAT(2,:);
        Sz_cone(j,:)  = CMAT(3,:);
    end

    for i = 1:SenLines
        for j = 1:size(rangeToF,1)
        %transalte and rotate cone matrix to sensor location
        CMAT=rotz(ang_set(i))*[Sx_cone(j,:);SrangeToF(j,:);Sz_cone(j,:)]+SPlace(:,:,i);
        x_cone_S(j,:,i) = CMAT(1,:);
        rangeToF_S(j,:,i)= CMAT(2,:);
        z_cone_S(j,:,i)  = CMAT(3,:);
        end
    end
    
    % create a gondola sensor bay
    %first determine offset
    gondolaParams = airship.gondolaSetup;
    gond_height = gondolaParams.height; %gondola height
    offset = radius + gond_height/2 + Sensor.ToFSensor.Size(2)/2;
    %define number of sensors and the number of the 
    num_gondsensors = gondolaParams.numberOfSensors;       %number of sensors on the gondola
    gond_ang = 360/(gondolaParams.numberOfSensors+balloonParams.numberOfSensors);             % angle between sensors
    gond_set_ang = gondolaParams.sensorZPlaneRot;         % angle of inclination of the sensor on thhhe gondola
    gond_ang_set = ang_set-gond_ang; % want to determine angle rotation about the z- axis 
    foc_len = gondolaParams.radius;%Sensor.ToFSensor.Size(2)/2 * 1/tand(15); % length from focus to centre of the sensor
    gond_place   = [foc_len*cosd(gond_ang_set+90);foc_len*sind(gond_ang_set+90);-offset*ones(1,num_gondsensors)];
    for i = 1:num_gondsensors
    GNMAT(:,:,i)= rotz(gond_ang_set(i))*rotx(gond_set_ang)*SMAT + gond_place(:,i);
    end

    for i = 1:num_gondsensors
        for j = 1:size(rangeToF,1)
            %transalte and rotate cone matrix to sensor location
            CMAT=rotz(gond_ang_set(i))*rotx(gond_set_ang)*[x_cone(j,:);rangeToF(j,:);z_cone(j,:)]+ gond_place(:,i);
            Gx_cone(j,:,i) = CMAT(1,:);
            GrangeToF(j,:,i)= CMAT(2,:);
            Gz_cone(j,:,i)  = CMAT(3,:);
        end
    end
    
    
    
    % plot visual
    figure('Name', 'Visual');
    grid on
    hold on
    axis([-5 5 -5 5 -5 5]);
    plot3([0 0],[0 0],[-radius radius],'r');
    %plot sensors on the envelope
    for i = 1:SenLines
        plot3(NMAT(1,:,i),NMAT(2,:,i),NMAT(3,:,i),'b');
        plot3(SNMAT(1,:,i),SNMAT(2,:,i),SNMAT(3,:,i),'k');
        mesh(x_cone_S(:,:,i),rangeToF_S(:,:,i),z_cone_S(:,:,i));
    end
    %plot sensors on gondola
    for i = 1:num_gondsensors
        mesh(Gx_cone(:,:,i),GrangeToF(:,:,i),Gz_cone(:,:,i));
        plot3(GNMAT(1,:,i),GNMAT(2,:,i),GNMAT(3,:,i),'k')
    end

    xlabel('x');
    ylabel('y');
    zlabel('z');
    
    
    
    sensorLines.balloon.x = x_cone_S;
    sensorLines.balloon.y = rangeToF_S;
    sensorLines.balloon.z = z_cone_S;
    
    sensorLines.gondola.x = Gx_cone;
    sensorLines.gondola.y = GrangeToF;
    sensorLines.gondola.z = Gz_cone;
end