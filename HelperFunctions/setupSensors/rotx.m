function out = rotx(ang)
    out = [1 0 0;0 cosd(ang) -sind(ang);0 sind(ang) cosd(ang)];
end