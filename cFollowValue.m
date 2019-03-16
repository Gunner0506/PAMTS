function value = cFollowValue( dd )
%calculate the PoF
global r
par = 1/3;
if dd<=r*par
value = 1;
elseif dd<=r
    value  = (1-dd/r)+par;
else
    value  = 0;
end

