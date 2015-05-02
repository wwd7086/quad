% generate a new map based on fan location
% map: 15*15
% fp1,fp2: 2*1  represented as 3*3 block in map
function map = gen_map(fp1, fp2, mapSize, fanSize)

%init
map = zeros(mapSize)';

%place fan
map = place_fan(map,fp1,mapSize,fanSize);
map = place_fan(map,fp2,mapSize,fanSize);

end

function map = place_fan(map,fp, mapSize, fanSize)
%constrain fp1 fp2 value
if all(fp>0) && all(fp<=mapSize)
    xsInd = max(fp(2)-fanSize,1);
    xeInd = min(fp(2)+fanSize,mapSize);
    ysInd = max(fp(1)-fanSize,1);
    yeInd = min(fp(1)+fanSize,mapSize);
    
    map(xsInd:xeInd,ysInd:yeInd) = 1;
end
end