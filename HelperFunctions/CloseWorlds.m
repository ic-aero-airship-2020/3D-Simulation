function CloseWorlds()
    out = vrwho;
    for i=1:length(out)
        while (get(out(i),'opencount')~=0)
            close(out(i));
        end
        delete(out(i));
    end
end