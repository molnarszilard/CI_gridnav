function maxQ=maxq(Q,i,j,k,size)
q0=Q(i,j,k);
if i>1 || (i==1 && k~=1)  % nem az elso oszlopban vagyok
    q1=Q(i-1,j,k);
else
    q1=-100;
end
if i<size(1) || (i==size(1) && k~=2)  % lehet menni "a falig", nem az 5. oszlopban vagyok
    q2=Q(i+1,j,k);
else
    q2=-100;
end
if j>1 || (j==1 && k~=3)
    q3=Q(i,j-1,k);
else
    q3=-100;
end
if j<size(2) || (j==size(2) && k~=4) 
    q4=Q(i,j+1,k);
else
    q4=-100;
end
maxQ=max([q0,q1,q2,q3,q4]);
end