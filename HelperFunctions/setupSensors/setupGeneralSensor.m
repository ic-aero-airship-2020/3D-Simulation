function Sensor = setupGeneralSensor(dayRange, nightRange, FoV, sensorSize)
    %define sensor types

    %Sensor.SensorName.
    % -Range.
        % - day = day time range
        % - night = night time range
    % - FoV = field of vision
    % - Size = [width length]
    % - Centre = [where is assumed possition of sensor on breakout]
    % - Shape  = [coordinate set 1; .......; coordinate set 4] (starts from top left rotates clockwise)


    %sensor 1  = ToF Sensor

    Sensor.ToFSensor.Range.day   = dayRange;
    Sensor.ToFSensor.Range.night = nightRange;
    Sensor.ToFSensor.FoV  = FoV;
    Sensor.ToFSensor.Size = sensorSize;   
    Sensor.ToFSensor.Centre = [0 0];
    Sensor.ToFSensor.Shape = [-Sensor.ToFSensor.Size(1)/2 Sensor.ToFSensor.Size(2)/2;...
                              Sensor.ToFSensor.Size(1)/2 Sensor.ToFSensor.Size(2)/2;...
                              Sensor.ToFSensor.Size(1)/2 -Sensor.ToFSensor.Size(2)/2;...
                              -Sensor.ToFSensor.Size(1)/2 -Sensor.ToFSensor.Size(2)/2]';
end