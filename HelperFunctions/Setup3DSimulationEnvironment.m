function virtualWorld = Setup3DSimulationEnvironment(modelPath,sensorLines)
    % ###### Closing all vrworlds ######
    CloseWorlds();

    
    % ###### Loading 3D vrworld for simulation ######
    virtualWorld = vrworld(modelPath);
    open(virtualWorld);
    
    
    % ###### Airship node ######
    airshipNode = virtualWorld.Airship;
    airshipNode.translation = [0, 2.5, 0];
    
    % ###### Sensors node ######
    balloonSensors = sensorLines.balloon;
    gondolaSensors = sensorLines.gondola;

    coordPoints1 = [balloonSensors.x(1,1,1),balloonSensors.z(1,1,1),balloonSensors.y(1,1,1);balloonSensors.x(2,:,1)',balloonSensors.z(2,:,1)',balloonSensors.y(2,:,1)'];
    coordPoints2 = [balloonSensors.x(1,1,2),balloonSensors.z(1,1,2),balloonSensors.y(1,1,2);balloonSensors.x(2,:,2)',balloonSensors.z(2,:,2)',balloonSensors.y(2,:,2)'];
    coordPoints3 = [balloonSensors.x(1,1,3),balloonSensors.z(1,1,3),balloonSensors.y(1,1,3);balloonSensors.x(2,:,3)',balloonSensors.z(2,:,3)',balloonSensors.y(2,:,3)'];
    
    coordPoints4 = [gondolaSensors.x(1,1,1),gondolaSensors.z(1,1,1),gondolaSensors.y(1,1,1);gondolaSensors.x(2,:,1)',gondolaSensors.z(2,:,1)',gondolaSensors.y(2,:,1)'];
    coordPoints5 = [gondolaSensors.x(1,1,2),gondolaSensors.z(1,1,2),gondolaSensors.y(1,1,2);gondolaSensors.x(2,:,2)',gondolaSensors.z(2,:,2)',gondolaSensors.y(2,:,2)'];
    coordPoints6 = [gondolaSensors.x(1,1,3),gondolaSensors.z(1,1,3),gondolaSensors.y(1,1,3);gondolaSensors.x(2,:,3)',gondolaSensors.z(2,:,3)',gondolaSensors.y(2,:,3)'];
    
    coordIndex = zeros(3,(size(coordPoints1,1)-1));
    coordIndex(2,:) = 1:(size(coordPoints1,1)-1);
    coordIndex(3,:) = -1;
    coordIndex = coordIndex(:);
    
    sensorConeCoord1 = virtualWorld.Cone_0001;
    sensorConeCoord1.coordIndex = coordIndex;
    sensorConeCoord1.coord.point = coordPoints1;
    sensorConeVisualCoord1 = virtualWorld.Cone_Visual_0001;
    sensorConeVisualCoord1.coordIndex = coordIndex;
    sensorConeVisualCoord1.coord.point = coordPoints1;
    
    sensorConeCoord2 = virtualWorld.Cone_0002;
    sensorConeCoord2.coordIndex = coordIndex;
    sensorConeCoord2.coord.point = coordPoints2;
    sensorConeVisualCoord2 = virtualWorld.Cone_Visual_0002;
    sensorConeVisualCoord2.coordIndex = coordIndex;
    sensorConeVisualCoord2.coord.point = coordPoints2;
    
    sensorConeCoord3 = virtualWorld.Cone_0003;
    sensorConeCoord3.coordIndex = coordIndex;
    sensorConeCoord3.coord.point = coordPoints3;
    sensorConeVisualCoord3 = virtualWorld.Cone_Visual_0003;
    sensorConeVisualCoord3.coordIndex = coordIndex;
    sensorConeVisualCoord3.coord.point = coordPoints3;
    
    sensorConeCoord4 = virtualWorld.Cone_0004;
    sensorConeCoord4.coordIndex = coordIndex;
    sensorConeCoord4.coord.point = coordPoints4;
    sensorConeVisualCoord4 = virtualWorld.Cone_Visual_0004;
    sensorConeVisualCoord4.coordIndex = coordIndex;
    sensorConeVisualCoord4.coord.point = coordPoints4;
    
    sensorConeCoord5 = virtualWorld.Cone_0005;
    sensorConeCoord5.coordIndex = coordIndex;
    sensorConeCoord5.coord.point = coordPoints5;
    sensorConeVisualCoord5 = virtualWorld.Cone_Visual_0005;
    sensorConeVisualCoord5.coordIndex = coordIndex;
    sensorConeVisualCoord5.coord.point = coordPoints5;
    
    sensorConeCoord6 = virtualWorld.Cone_0006;
    sensorConeCoord6.coordIndex = coordIndex;
    sensorConeCoord6.coord.point = coordPoints6;
    sensorConeVisualCoord6 = virtualWorld.Cone_Visual_0006;
    sensorConeVisualCoord6.coordIndex = coordIndex;
    sensorConeVisualCoord6.coord.point = coordPoints6;
    
    % ###### Line Sensors node ######
    balloonLineSensors = sensorLines.balloon_line;
    gondolaLineSensors = sensorLines.gondola_line;
    
    
    coordLinePoints1 = [balloonLineSensors.x(1,1,1),balloonLineSensors.z(1,1,1),balloonLineSensors.y(1,1,1);balloonLineSensors.x(2,:,1)',balloonLineSensors.z(2,:,1)',balloonLineSensors.y(2,:,1)'];
    coordLinePoints2 = [balloonLineSensors.x(1,1,2),balloonLineSensors.z(1,1,2),balloonLineSensors.y(1,1,2);balloonLineSensors.x(2,:,2)',balloonLineSensors.z(2,:,2)',balloonLineSensors.y(2,:,2)'];
    coordLinePoints3 = [balloonLineSensors.x(1,1,3),balloonLineSensors.z(1,1,3),balloonLineSensors.y(1,1,3);balloonLineSensors.x(2,:,3)',balloonLineSensors.z(2,:,3)',balloonLineSensors.y(2,:,3)'];
    
    coordLinePoints4 = [gondolaLineSensors.x(1,1,1),gondolaLineSensors.z(1,1,1),gondolaLineSensors.y(1,1,1);gondolaLineSensors.x(2,:,1)',gondolaLineSensors.z(2,:,1)',gondolaLineSensors.y(2,:,1)'];
    coordLinePoints5 = [gondolaLineSensors.x(1,1,2),gondolaLineSensors.z(1,1,2),gondolaLineSensors.y(1,1,2);gondolaLineSensors.x(2,:,2)',gondolaLineSensors.z(2,:,2)',gondolaLineSensors.y(2,:,2)'];
    coordLinePoints6 = [gondolaLineSensors.x(1,1,3),gondolaLineSensors.z(1,1,3),gondolaLineSensors.y(1,1,3);gondolaLineSensors.x(2,:,3)',gondolaLineSensors.z(2,:,3)',gondolaLineSensors.y(2,:,3)'];
    
    coordLineIndex = zeros(3,(size(coordLinePoints1,1)-1));
    coordLineIndex(2,:) = 1:(size(coordLinePoints1,1)-1);
    coordLineIndex(3,:) = -1;
    coordLineIndex = coordLineIndex(:);
    
    sensorLineCoord1 = virtualWorld.Cone_line_0001;
    sensorLineCoord1.coordIndex = coordLineIndex;
    sensorLineCoord1.coord.point = coordLinePoints1;
    sensorLineVisualCoord1 = virtualWorld.Cone_Visual_line_0001;
    sensorLineVisualCoord1.coordIndex = coordLineIndex;
    sensorLineVisualCoord1.coord.point = coordLinePoints1;
    
    sensorLineCoord2 = virtualWorld.Cone_line_0002;
    sensorLineCoord2.coordIndex = coordLineIndex;
    sensorLineCoord2.coord.point = coordLinePoints2;
    sensorLineVisualCoord2 = virtualWorld.Cone_Visual_line_0002;
    sensorLineVisualCoord2.coordIndex = coordLineIndex;
    sensorLineVisualCoord2.coord.point = coordLinePoints2;
    
    sensorLineCoord3 = virtualWorld.Cone_line_0003;
    sensorLineCoord3.coordIndex = coordLineIndex;
    sensorLineCoord3.coord.point = coordLinePoints3;
    sensorLineVisualCoord3 = virtualWorld.Cone_Visual_line_0003;
    sensorLineVisualCoord3.coordIndex = coordLineIndex;
    sensorLineVisualCoord3.coord.point = coordLinePoints3;
    
    sensorLineCoord4 = virtualWorld.Cone_line_0004;
    sensorLineCoord4.coordIndex = coordLineIndex;
    sensorLineCoord4.coord.point = coordLinePoints4;
    sensorLineVisualCoord4 = virtualWorld.Cone_Visual_line_0004;
    sensorLineVisualCoord4.coordIndex = coordLineIndex;
    sensorLineVisualCoord4.coord.point = coordLinePoints4;
    
    sensorLineCoord5 = virtualWorld.Cone_line_0005;
    sensorLineCoord5.coordIndex = coordLineIndex;
    sensorLineCoord5.coord.point = coordLinePoints5;
    sensorLineVisualCoord5 = virtualWorld.Cone_Visual_line_0005;
    sensorLineVisualCoord5.coordIndex = coordLineIndex;
    sensorLineVisualCoord5.coord.point = coordLinePoints5;
    
    sensorLineCoord6 = virtualWorld.Cone_line_0006;
    sensorLineCoord6.coordIndex = coordLineIndex;
    sensorLineCoord6.coord.point = coordLinePoints6;
    sensorLineVisualCoord6 = virtualWorld.Cone_Visual_line_0006;
    sensorLineVisualCoord6.coordIndex = coordLineIndex;
    sensorLineVisualCoord6.coord.point = coordLinePoints6;
    
    
    % ###### Showing world ######
%     virtualWorldFigure = vrfigure(virtualWorld);
end