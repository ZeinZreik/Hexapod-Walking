function q=IKdLeg(i,Px_,Py_,Pz_,alpha_,beta_,gamma_,Px,Py,Pz,alpha,beta,gamma,LL)
    %q(1:6,1:6)
    global L0;
    global L1;
    global L2;
    global L3;
    global q0;
    L0=100; L1=53; L2=68; L3=107;
    q0=[pi/3 0 -pi/3 -2*pi/3 -pi 2*pi/3];
    
    sa=sin(alpha);
    sb=sin(beta);
    sy=sin(gamma);
    ca=cos(alpha);
    cb=cos(beta);
    cy=cos(gamma);

    c0=cos(q0(i));
    s0=sin(q0(i));
    X_0i=-Px_-L0*(alpha_*(-sa*cb*c0-s0*(sa*sb*sy+ca*cy))+beta_*(-c0*ca*sb+s0*ca*cb*sy)+gamma_*s0*(ca*sb*cy+sa*sy));
    Y_0i=-Py_-L0*(alpha_*(c0*ca*cb+s0*(ca*sb*sy-sa*cy))+beta_*(-c0*sa*sb+s0*sa*cb*sy)+gamma_*s0*(sa*sb*cy-ca*sy));
    Z_0i=-Pz_+L0*(beta_*(c0*cb-s0*sb*sy)+beta_*s0*cb*cy);
    
    
    %%%%%%%
    %1
    %R0 coordinate
    x0(i)=Px+L0*(c0*ca*cb+s0*(ca*sb*sy-sa*cy));
    y0(i)=Py+L0*(c0*sa*cb+s0*(sa*sb*sy+ca*cy));
    z0(i)=Pz-L0*(c0*sb-s0*cb*sy);
    %x0(i)=Px+ L0*(cos(q0(i))+sin(q0(i)));
    %y0(i)=Py+ L0*( + sin(q0(i))*(1 ) ) ;
    %z0(i)=Pz;% - L0*( cos(q0(i))*sin(beta)+sin(q0(i))*cos(beta)*sin(gamma) );

    %2
    %find rO0_OL this writtin in R0i frame
    xL(i)=LL*c0-x0(i);
    yL(i)=LL*s0-y0(i);
    zL(i)=0-z0(i);
    
    %3
    %IK flight without L0
    %q(leg id, angle )
    
    q(1)=atan2(yL(i),xL(i))-q0(i);
    if(q(1)>pi)
         q(1)=q(1)-2*pi;
    end
    p1=sqrt(xL(i)*xL(i)+yL(i)*yL(i))-L1;
    p2=zL(i)*zL(i)-L2*L2-L3*L3;
        
    q(3)=-(acos((p1^2+p2)/(2*L2*L3)));
        
    p3=(L3*sin(q(3)))^2+(L2+L3*cos(q(3)))^2;
    m1=L2+L3*cos(q(3));
    m2=L3*sin(q(3));
    c01=cos(q0(i)+q(1));
    p=xL(i)/c01;
    S2=(m1*zL(i)- m2*(p-L1))/p3;
    C2=(m2*zL(i)+ m1*(p-L1))/p3;
   
    q(2)=atan2(S2,C2);
    
    %%%%%%%%%%%%%%%%
    t01=tan(q0(i)+q(1));
    t23=tan(q(2)+q(3));
    s01=sin(q0(i)+q(1));
    c23=cos(q(2)+q(3));
    s2=sin(q(2));
    c2=cos(q(2));
    
    q(5)=(c01*(X_0i+Y_0i*t01)+Z_0i*t23 )/(L2*(c2*t23-s2));
    q(4)=(Y_0i-s01*(-q(5)*L2*s2-(Z_0i-q(5)*L2*c2)*t23) )/(c01*(L1+L2*c2+L3*c23));
    q(6)=(Z_0i-q(5)*L2*c2)/(L3*c23)-q(5);
end