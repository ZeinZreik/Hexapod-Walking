function q = Walk(time)
    global q1;
    global q2;
    counter = mod(int32(time/0.5*41),41)+1;
    Q1 = reshape(q1(counter,:,:),[3 6]);
    Q2 = reshape(q2(counter,:,:),[3 6]);
    Q1 = reshape(Q1',[1 18]);
    Q2 = reshape(Q2',[1 18]);
    q = [Q1 Q2];
end