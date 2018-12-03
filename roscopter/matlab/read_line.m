function ret = read_line(file)
    ret = strsplit(fgetl(file));
    ret = cellfun(@str2num,ret(2:end),'un',0).';
    ret = [ret{:}];
end
