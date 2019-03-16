function At_West(m)
global  target;

mx = target(1,m);
my = target(2,m);
max_temp = randi([1,6],1,1);

switch max_temp
    case 1
        return;
    case 2
        target(:,m) = [mx-1;my];
        return;
    case 3
        target(:,m) = [mx-1;my+1];
        return;
    case 4
        target(:,m) = [mx;my+1];
        return;
    case 5
        target(:,m) = [mx+1;my+1];
        return;
    case 6
        target(:,m) = [mx+1;my];
        return;
end
end