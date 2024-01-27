function q =IKd(in)
    Px_ = in(1);
    Py_ = in(2);
    Pz_ = in(3);
    alpha_ = in(4);
    beta_ = in(5);
    gamma_ = in(6);
    Px = in(7);
    Py = in(8);
    Pz = in(9);
    alpha = in(10);
    beta = in(11);
    gamma = in(12);
    LL = in(13);
    %this function will calculate the angular velocity of the wholl robot
    
    %q (i leg ; 123 angles 456 velocity)
    for i=1:6
        q(:,i)=IKdLeg(i,Px_,Py_,Pz_,alpha_,beta_,gamma_,Px,Py,Pz,alpha,beta,gamma,LL)';
    end
end