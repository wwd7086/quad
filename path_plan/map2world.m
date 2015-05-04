% convert from map to world coordinate
function pw = map2world(pm, map_scale, worldRect)
pw = pm/map_scale;
pw = bsxfun(@minus, pw, 1/(2*map_scale));
pw(:,1) = bsxfun(@plus, pw(:,1), worldRect(1));
pw(:,2) = bsxfun(@plus, pw(:,2), worldRect(3));
end