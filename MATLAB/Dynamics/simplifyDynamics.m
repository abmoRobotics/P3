function [tauTemp] = simplifyDynamics()
%SUBVAL Summary of this function goes here
%   Detailed explanation goes here
syms dq1(t) dq2(t) dq3(t) dq4(t) dq5v(t) dq5h(t) q1(t) q2(t) q3(t) q4(t) q5v(t) q5h(t) ddq1(t) ddq2(t) ddq3(t) ddq4(t) ddq5v(t) ddq5h(t) m1 m2 m3 m4 m5 g Q1 Q2 Q3 Q4 Q5v Q5h DQ1 DQ2 DQ3 DQ4 DQ5v DQ5h DDQ1 DDQ2 DDQ3 DDQ4 DDQ5v DDQ5h
  ddq=[ddq1(t); ddq2(t); ddq3(t); ddq4(t); ddq5v(t); ddq5h(t)];
  dq=[dq1(t); dq2(t); dq3(t); dq4(t); dq5v(t); dq5h(t)];
  q=[q1(t);q2(t);q3(t);q4(t);q5v(t);q5h(t)];
  Q=[Q1;Q2;Q3;Q4;Q5v;Q5h];
  DQ=[DQ1;DQ2;DQ3;DQ4;DQ5v;DQ5h];
  DDQ=[DDQ1;DDQ2;DDQ3;DDQ4;DDQ5v;DDQ5h];
  mMatrix=[m1;m2;m3;m4;m5; m5];
  m = [189.97;240.32;109.23;188.41;33.39;33.39];
  m = m*(1/1000);
  % m= [m1,m2,m3,m4,m5,m5];
  

    qVal = [0;0;0;0;0;0];
    tauTemp = deriveDynamics();
    
    %For gravity + coriolis:    Q(i)=Q(i)   DQ(i)=DQ(i) DDQ(i)=0      g=-9.82
    %For coriolis:              Q(i)=Q(i)   DQ(i)=DQ(i) DDQ(i)=0      g=0
    %For MassMatrix + coriolis: Q(i)=Q(i)   DQ(i)=DQ(i) DDQ(i)=DDQ(i) g=0
    
    for index = 1:6
        

        
        for i = 1:6
        tauTemp(index) = subs(tauTemp(index),q(i),Q(i));        %Position
        tauTemp(index) = subs(tauTemp(index),dq(i),DQ(i));      %Velocity
        tauTemp(index) = subs(tauTemp(index),ddq(i),DDQ(i));    %Acceleration
        tauTemp(index) = subs(tauTemp(index),mMatrix(i),m(i));  %Mass
        
        end
        tauTemp(index) = subs(tauTemp(index),g,0);          %Gravity
        
        
    end
    tauTemp = tauTemp;
end

