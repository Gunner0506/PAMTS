clc;clear
global X Y M N target uav r timeStamp v uIndex tIndex eWeight fWeight Flag flag 
for group = 2:7
    cnt =5 ;%runs of each group
    res = zeros(cnt,1);%record the result of each simulation run
    size = 400;%the size of the search region is 400*400
    X = size;%the length of the search region
    Y = size;%the width of the search region
    M = 40;%the number of movcing targets
    r = 26;%SR of each UAV
    v = 2;%maxinum velocity of each UAV
    dc = 100;%DTR of each UAV
    for k = 1:cnt
        target =  randi([40,360],2,M);%the locations of the targets
        N = 5*group;%the number of the UAVs
        uav =  randi([40,360],2,N);%the lcoations of the UAVs
        times = 500;%runtime of each simulation run
        sum = zeros(times,1);%record the total targets observed by all UAVs 
        timeStamp = zeros(X,Y,N);%timestamps of the cells
        ratio = M/N;
        for time = 1:times
            tIndex = ones(N,M)*100000;%record the distance among UAVs and targets
            uIndex = ones(N,M)*-1;%record the distance between the observed target and UAV
            distanceOfUAV = zeros(N,N);%record the distance among UAVs
            flag = zeros(N,3);%1st column:the serial number of the sub team of UAV;2nd column:the total number of targets observed by the sub team;3rd:the number of UAVs in sub team
            cntn = zeros(1,N);%the number of UAVs observed by each UAV
            eWeight = zeros(1,N);%weight coefficients of explore intention
            fWeight = zeros(1,N);%weight coefficients of follow intention
            
            %calculate the distance between the observed targets and UAVs
            for m = 1:M
                tx = target(1,m);
                ty = target(2,m);
                for n = 1:N
                    ux = uav(1,n);
                    uy = uav(2,n);
                    d = countDistance(ux,uy,tx,ty);
                    if d<=r
                        tIndex(n,m) = d;
                    end
                end
            end
            
            %determine the tracker of each observed target
            for m = 1:M
                tag = tIndex(:,m);%取目标对应的那一列
                [minD,minIndex] = min(tag);%有可能多个最小
                if minD<100000
                    cntn(minIndex) = cntn(minIndex) +1;%该目标被对应的无人机tag
                    uIndex(minIndex,m) = minD;%记录下目标的归属
                end
            end
            
            %calculate the distance among UAVs
            for n = 1:N
                ux = uav(1,n);
                uy = uav(2,n);
                for n1 = 1:N
                    ux1 = uav(1,n1);
                    uy1 = uav(2,n1);
                    d = countDistance(ux,uy,ux1,uy1);
                    distanceOfUAV(n,n1) = d;%在对应的矩阵中记录
                end
            end
            
            %use the "DFS" method to determine the sub team of each UAV
            %and the observation summary of each UAV
            Flag = 1;%the serial number of sub team
            visit =zeros(N,1);
            for n = 1:N
                num = 0;
                total = 0;
                if visit(n)==0
                    top = 1;
                    visit(n) = 1;
                    stack(top) = n;
                    num = num + cntn(n);
                    total = total+1;
                    flag(n,1) = Flag;
                    Flag = Flag +1;
                    while top~=0
                        pre_len=length(stack);
                        i = stack(top);         
                        for j = 1:N
                            d = distanceOfUAV(i,j);
                            if visit(j)==0 && d<=dc
                                top = top+1;
                                stack(top) = j;
                                visit(j) = 1;
                                flag(j,1) = Flag-1;
                                num = num+cntn(j);
                                total = total+1;
                                break;
                            end
                        end
                        if length(stack)==pre_len
                            stack(top) = [];
                            top = top-1;
                        end
                    end
                    for nn = 1:N
                        if flag(nn,1) == Flag-1
                            flag(nn,2) = num;
                            flag(nn,3) = total;
                        end
                    end
                end
            end
            
            %record the total number of observed targets at this time step
            for n = 1:N
                sum(time) = sum(time)+cntn(n);
            end
            
       %calculate the weight coefficients of follow and explore intentions
            for n = 1:N
                gWeight = flag(n,2)/(M/N*flag(n,3));
                if cntn(n) >= ratio
                    fWeight(n) = 1*gWeight;
                    eWeight(n) = 0;
                elseif cntn(n)>=0.75*ratio
                    fWeight(n) = 0.75*gWeight;
                    eWeight(n) = 0.25*(1-gWeight);
                elseif cntn(n)>=0.5*ratio
                    fWeight(n) = 0.5*gWeight;
                    eWeight(n) = 0.5*(1-gWeight);
                elseif cntn(n)>=0.25*ratio
                    fWeight(n) = 0.25*gWeight;
                    eWeight(n) = 0.75*(1-gWeight);
                else
                    fWeight(n) = 0;
                    eWeight(n) = 1*(1-gWeight);
                end
            end
            updateTimeStamp(time);%upadte the timestamp of each cell
            caculateValue(time);%profit calculation and path planning
            target_move();%the movements of the targets
        end
        res(k) =  mean(sum(:,1));
    end
    %show the result of each group
    disp('...............................................................')
    disp(N);
    disp(mean(res(:,1)))
end
