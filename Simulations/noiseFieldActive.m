function tf = noiseFieldActive(v)

tf = ~isempty(v) && any(v(:) ~= 0);

end
