function cfg = str2cfg(str, keywords)
% String configuration to structure
if isstruct(str)
    cfg = str;
    return;
end
if isstruct(keywords)    
    keywords = fieldnames(keywords); 
end

slen = length(str);

cfg = struct;
for i = 1:length(keywords)
    kw = keywords{i}; kwlen = length(kw);
    indices = strfind(str, keywords{i});
    if isempty(indices), continue; end      % no setting found
    accept = 0;
    for j = 1:length(indices)
        ix = indices(j); after = ix + kwlen; before = ix - 1;
        if ((after > slen) || (str(after) == '=') || (str(after) == ' ')) ...
                && ((before < 1) || (str(before) == ' '))
            accept = 1;
            break;
        end
    end
    if ~accept, continue; end
    % if no value given, interpreted as boolean true
    if (after > slen) || (str(after) == ' ')
        cfg.(kw) = true;
        continue;
    end
    % otherwise, parse after equal
    val = strtok(str(after+1:end), ' ');

    if ~any(val(1) == ['a':'z' 'A':'Z']) && ~isempty(str2num(val))
        val = str2num(val);
    elseif val(1) == '{',           % cell array
        try val = eval(val);
        catch   
        end
    end
    cfg.(kw) = val;
end;
