function msg = copyImage(msgStruct)
    msg = rosmessage(msgStruct.MessageType);
    fNames = fieldnames(msg);
    for k = 1:numel(fNames)
        if ~any(strcmp(fNames{k}, {'MessageType', 'Header'}))
            msg.(fNames{k}) = msgStruct.(fNames{k});
        end
    end
end