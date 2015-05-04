% convert from world to map coordniate
function pm = world2map(pw, map_scale, worldRect)
pw(:,1) = bsxfun(@minus, pw(:,1), worldRect(1));
pw(:,2) = bsxfun(@minus, pw(:,2), worldRect(3));	
pm = ceil(bsxfun(@plus, pw*map_scale, 0.000000001));
end