function updateTimeStamp(time)
global uav N X Y r timeStamp Flag flag
for n = 1:N
    ux = uav(1,n);%the x position of the UAV
    uy = uav(2,n);%the y position of the UAV
    %deal with the boundary situations
    if ux>r
        x1 = ux-r;
    else
        x1 = 1;
    end
    if ux+r<=X
        x2 = ux+r;
    else
        x2 = X;
    end
    if uy>r
        y1 = uy-r;
    else
        y1 = 1;
    end
    if uy+r<=Y
        y2 = uy+r;
    else
        y2 = Y;
    end
    %update the timestamp in local map
    for x = x1:x2
        for y = y1:y2
            d= countDistance( x,y,ux,uy );
            if d<=r
                timeStamp(x,y,n) = time;
            end
        end
    end
end

 %update the timestamp in information merging process
for k = 1:Flag-1
    tmp = zeros(X,Y);
    for n = 1:N
        if flag(n,1) == k
            for x = 1:X
                for y = 1:Y
                    if timeStamp(x,y,n)>tmp(x,y)
                        tmp(x,y) = timeStamp(x,y,n);
                    end
                end
            end
        end
    end
    for n = 1:N
        if flag(n,1) == k
            timeStamp(:,:,n) = tmp;
        end
    end
end


end

