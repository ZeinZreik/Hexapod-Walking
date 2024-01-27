function q =IKdTripod(Px_,Py_,Pz_,alpha_,beta_,gamma_,Px,Py,Pz,alpha,beta,gamma,LL,tripod,oneLeg)
%this function will calculate the angular velocity of the wholl robot
%q (i leg ; 123 angles 456 velocity)
if(oneLeg==1)
    i=1;%return q(1,6)
    if(tripod==1)
        i=2;
    end
    q=IKdLeg(i,Px_,Py_,Pz_,alpha_,beta_,gamma_,Px,Py,Pz,alpha,beta,gamma,LL);

else
    for i=1:3
        if(tripod==2)
            q(i,:)=IKdLeg(2*(i),Px_,Py_,Pz_,alpha_,beta_,gamma_,Px,Py,Pz,alpha,beta,gamma,LL);
        else
            q(i,:)=IKdLeg(2*(i-1)+1,Px_,Py_,Pz_,alpha_,beta_,gamma_,Px,Py,Pz,alpha,beta,gamma,LL);
        end
        %
    end
end
end

