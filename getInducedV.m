%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Function to determine induced voltage with the given the normalisind distance,
%position of transmitter and receiver.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function M = getInducedV(Pt,Pr,d)
    xr=Pr(1);
    yr=Pr(2);
    zr=Pr(3);
    thetar=Pr(4);
    phir=Pr(5);
    
    xt=Pt(1);
    yt=Pt(2);
    zt=Pt(3);
    thetat=Pt(4);
    phit=Pt(5);
    
    [mxr,myr,mzr]=sph2cart(thetar,phir,1);      %Obtaining unit vectors of reciever from spherical coordinates
    [mxt,myt,mzt]=sph2cart(thetat,phit,1);      %Obtaining unit vector of transmitter from spherical coordinates

    mr=[mxr myr mzr];
    mt=[mxt myt mzt];
    mrc=mr/norm(mr);
    mtc=mt/norm(mt);
    rr=[xr/d yr/d zr/d];        %nomralised coordiantes of reciever
    rkt=[xt/d yt/d zt/d];       %nomralised coordiantes of transmitter
    Rk=rr-rkt;                  %Vector distance between transmitter and reciever
    mRk=norm(Rk);
    wa=4.33e6;                  % Constant given
    mu=(4*pi)*10^(-7);          % Permitivity of free space
    %Gradient of induced voltage with respect to transmitter position.
    DVk=-1i*wa*(mu/(4*pi))*((15*(((dot(mtc,Rk))*dot(mrc,Rk).*Rk)/(mRk^7)))-3*((dot(mtc,Rk)*mrc)+((dot(mrc,Rk))*mtc+(dot(mtc,mrc)).*Rk)/(mRk^5))+3*((dot(mrc,Rk).*Rk)/(mRk^5))-((mrc/(mRk^3))));

    M=DVk;                       % Return value of function 
   
end
% EOF