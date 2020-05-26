function [virtualWorld,virtualWorldFigure] = Setup3DSimulationEnvironment(modelPath)
    % ###### Closing all vrworlds ######
    out = vrwho;
    for i=1:length(out)
        while (get(out(i),'opencount')~=0)
            close(out(i));
        end
        delete(out(i));
    end

    
    % ###### Loading 3D vrworld for simulation ######
    virtualWorld = vrworld(modelPath);
    open(virtualWorld);
    
    
    % ###### Airship node ######
    airshipNode = virtualWorld.Airship;
    airshipNode.translation = [0, 1, 0];
    
    
    % ###### Showing world ######
    virtualWorldFigure = vrfigure(virtualWorld);
end