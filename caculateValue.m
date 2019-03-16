function caculateValue(time)
global uav N X Y v M target uIndex tIndex r timeStamp eWeight fWeight
eValue = zeros(X,Y);
tValue = zeros(X,Y);
for n = 1:N
    ux = uav(1,n);
    uy = uav(2,n);
    iValue = zeros(X,Y);
    if ux>v
        x1 = ux-v;
    else
        x1 = 1;
    end
    if ux+v<=X
        x2 = ux+v;
    else
        x2 = X;
    end
    if uy>v
        y1 = uy-v;
    else
        y1 = 1;
    end
    if uy+v<=Y
        y2 = uy+v;
    else
        y2 = Y;
    end
    fValue = zeros(X,Y);
    
    for x = x1:x2
        for y = y1:y2
            %calculate PoF
            d = countDistance( x,y,ux,uy );
            if d<=v
                for m = 1:M
                    tx = target(1,m);
                    ty = target(2,m);
                    dd= countDistance( x,y,tx,ty);
                    if dd<=r
                        if uIndex(n,m)~=-1
                            fValue(x,y) = fValue(x,y)+cFollowValue(dd);
                        else
                            if tIndex(n,m) >r
                                index = uIndex(:,m);
                                [a,b] = max(index);
                                if a>-1
                                    fValue(x,y) = fValue(x,y)+cFollowValue(dd);
                                end
                            end
                        end
                    end
                end
            end
            %calculate PoE     
            if d<=v
            if eValue(x,y) == 0
                if x>r
                    tmpx1 = x-r;
                else
                    tmpx1 = 1;
                end
                if x+r<=X
                    tmpx2 = x+r;
                else
                    tmpx2 = X;
                end
                if y>r
                    tmpy1 = y-r;
                else
                    tmpy1 = 1;
                end
                if y+r<=Y
                    tmpy2 = y+r;
                else
                    tmpy2 = Y;
                end
                for tmpx = tmpx1:tmpx2
                    for tmpy = tmpy1:tmpy2
                        if tValue(tmpx,tmpy) == 0
                            if timeStamp(tmpx,tmpy,n)==0 || time-timeStamp(tmpx,tmpy,n)>=5
                                tValue(tmpx,tmpy) = 1;
                            else
                                tValue(tmpx,tmpy) = (time - timeStamp(tmpx,tmpy,n))*1/5;
                            end
                        end
                        eValue(x,y) = eValue(x,y)+ tValue(tmpx,tmpy);
                    end
                end
            end
            end
        end
    end
    
    %normalization process
    maxF = max(max(fValue));
    evalueIndex = eValue(x1:x2,y1:y2);
    maxE = max(max(evalueIndex));
    for x = x1:x2
        for y = y1:y2
            if maxF==0
                f_value = 0;
            else
            f_value = fValue(x,y)/maxF;
            end
            if maxE==0
                e_value=0;
            else
            e_value =eValue(x,y)/maxE;
            end
            iValue(x,y)= fWeight(n)*f_value+eWeight(n)*e_value;
        end
    end
    
    %path planning
    ivalueIndex = iValue(x1:x2,y1:y2);
    maxval = max(max(ivalueIndex));
    [row, col] = find(maxval==ivalueIndex);%求出最大值对应的行和列，但是可能多个相同，取其中f_value最大的一个
    len = length(row);
    if len ==1
        uav(1,n) = row+x1-1;
        uav(2,n) = col+y1-1;
    else
        u_x = row(1);
        u_y = col(1);
        m_f = fValue(u_x,u_y);
        for l = 2:len
            if fValue(row(l),col(l))>m_f
                u_x = row(1);
                u_y = col(1);
                m_f = fValue(u_x,u_y);
            end
        end
        uav(1,n) = u_x+x1-1;
        uav(2,n) = u_y+y1-1;
    end
end
end


