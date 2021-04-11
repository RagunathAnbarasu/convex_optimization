%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Sensor selection in a sensor array placed in a Cartesian Grid using
%convex optimisation.have considered a planar sensor array consisting of
%525 sensors placed on a cartesian plan with  |x| ? 100 mm, |y| ? 120mm
%and a cell size of 10mm in each direction. The transmitter position is 
%considered as pt = [0 cm, 0 cm, 10 cm, 90?, 90?] and the receivers are
%placed perpendicular to the transmitter array. Further the domain under
%consideration is taken as ?p defined in x ? [?50, 50]mm,
%y ? [?50, 50]mm,z ? [100, 200] mm, ? ? [70?, 110?],  ? ? [70?, 110?]. 
%Therefore 121 sensors come under the range of consideration among 
%which 3 optimal sensor positions are obtained using CVX. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; clc;
N=121;              %# of Sensors under prescribed range

x=0;
y=0;
z=10;
theta=90;
phi=90;
pt=[x;y;z;theta;phi];   %Position of transmitter
%Planar sensor array configuration
xx=0;
yy=0;
kk=1;
for ii=-100:10:100
    for jj=-120:10:120
        xx=ii;
        yy=jj;
        hold on;

        figure(1)
        plot(xx,yy,"b s");
        xlim([-150 150]);
        ylim([-150 150]);
        title('Planar Sensor Arrangement');
        kk=kk+1;
    end
end
xx=0;
yy=0;
kk=1;
for ii=-50:10:50
    for jj=-50:10:50
        xx=ii;
        yy=jj;
        pr(1,kk)=xx;        %x- coordinates of recivers
        pr(2,kk)=yy;        %y- coordinates of recivers
        pr(3,kk)=0;         %z- coordinates of recivers
        pr(4,kk)=0;         %Theta of recivers
        pr(5,kk)=0;         %Phi of recivers
        figure(1)
        plot(xx,yy,"r o");
        xlim([-150 150]);
        ylim([-150 150]);
        kk=kk+1;
    end
end
hold off;
legend('Available sensors','Considered Sensor Range','Location','northwest');


d=10;       %Normalising distance of reciver from transmitter
k=N;

for i=1:N
    G(i,:)=getInducedV(pt,pr(:,i),d);   %Function to find induced voltage
end 
%size(G);
p = N;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% D-optimal design
%
%      minimise    -log det sum((lambda)*M))
%      subject to  sum(lambda)=1,  lambda >=0
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


Obj_fn=0;           % Objective function to be minimised
cvx_begin           % CVX program starts
  variable lambda(p)        % Variable to be optimised (weights)
  for i=1:N                 % Sum of N sensors
      Obj_fn=Obj_fn + lambda(i)*(G(i,:)'*G(i,:))
  end
  minimise (-det_rootn(Obj_fn));    % Minimise objective function 
  subject to
    sum(lambda) == 1;               % Sum(lambda)==1
    for i=1:N
        lambda(i)>=0                % lambda >= 0
    end  

cvx_end
lambdaD = lambda

p_x=[1:N];
figure(3)
plot(p_x,lambdaD)                       % Plot distribution of value of lambda for N sensors.
title('Distribution of weights');
xlabel('#Sensor considered')
ylabel('lambda')
legend('Distribution of lambda');
o=zeros(N,1);
for i=1:N
    if(lambdaD(i)>0.03)
        o(i)=i;
    end
end
kk=1;
for ii=-100:10:100
    for jj=-120:10:120
        xx=ii;
        yy=jj;
        figure(4)
        hold on;
        plot(xx,yy,"b s");
        xlim([-150 150]);
        ylim([-150 150]);
        title('Optimal Sensor Arrangement');
        kk=kk+1;
    end
end
kk=1;
for ii=-50:10:50
    for jj=-50:10:50
        xx=ii;
        yy=jj;
        figure(4)
        if(o(kk)>0)
           plot(xx,yy,"c x");           % Plot optimised sensor locations
        else
            plot(xx,yy,"r o");
        end
        xlim([-150 150]);
        ylim([-150 150]);        
        kk=kk+1;
    end
end
hold off;
legend('Available sensors','Optimal Sensors','Considered Sensor Range','Location','northwest');
%EOF