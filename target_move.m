function target_move()
global M target X Y;
for i = 1:M
    %at the corner of the search region
    if target(:,i) ==[1;1]
        At_NorthWest(i);
        continue;
    elseif target(:,i) == [1;Y]
        At_NorthEast(i);
        continue;
    elseif target(:,i)== [X;1]
        At_SouthWest(i);
        continue;
    elseif target(:,i) == [X;Y]
        At_SouthEast(i);
        continue;
    %at the edge of the search region
     elseif target(1,i) == 1
        At_North(i);
        continue;
    elseif target(1,i) == X
        At_South(i);
        continue;
    elseif target(2,i) == 1
        At_West(i);
        continue;
    elseif target(2,i) == Y
        At_East(i);
        continue;
    else
    %inside
        At_Inside(i);
        continue;
    end
end
end




