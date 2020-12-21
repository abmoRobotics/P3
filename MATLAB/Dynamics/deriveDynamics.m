function tau = deriveDynamics()
%DERIVEKINEMATICS Summary of this function goes here
%   Detailed explanation goes here
syms dq1(t) dq2(t) dq3(t) dq4(t) dq5v(t) dq5h(t) q1(t) q2(t) q3(t) q4(t) q5v(t) q5h(t) ddq1(t) ddq2(t) ddq3(t) ddq4(t) ddq5v(t) ddq5h(t) m1 m2 m3 m4 m5
  ddq=[ddq1(t); ddq2(t); ddq3(t); ddq4(t); ddq5v(t); ddq5h(t)];
  dq=[dq1(t); dq2(t); dq3(t); dq4(t); dq5v(t); dq5h(t)];
  q=[q1(t);q2(t);q3(t);q4(t);q5v(t);q5h(t)];
  mMatrix=[m1;m2;m3;m4;m5; m5];
  assumeAlso(dq1(t),'real')  
  assumeAlso(q1(t),'real') 
  assumeAlso(dq2(t),'real')  
  assumeAlso(q2(t),'real') 
  assumeAlso(dq3(t),'real')  
  assumeAlso(q3(t),'real') 
  assumeAlso(dq4(t),'real')  
  assumeAlso(q4(t),'real') 
  assumeAlso(dq5v(t),'real')  
  assumeAlso(q5v(t),'real') 
  assumeAlso(dq5h(t),'real')  
  assumeAlso(q5h(t),'real') 
  
 assumeAlso(ddq1(t),'real')  
  assumeAlso(ddq2(t),'real')  
  assumeAlso(ddq3(t),'real')  
  assumeAlso(ddq4(t),'real')  
  assumeAlso(ddq5v(t),'real')  
  assumeAlso(ddq5h(t),'real')     
    %% First distance to the center of mass is defined, this is measured in the local frame. (millimeter)
    S_lc1=[65.01;0;-14.46]/1000;
    S_lc2=[4.07;0;-109.29]/1000;
    S_lc3=[-8.98;53.41;-0.83]/1000;
    S_lc4=[58.16;0;-54.96]/1000;
    S_lc5v=[47.01;2.93;0]/1000;
    S_lc5h=[47.01;-2.93;0]/1000;
    %% Next distance from i frame to i+1 frame is defined, this is measured in the local frame. (millimeter)
    S_L1=[74;0;0]/1000;
    S_L2=[0;0;-171.7]/1000;
    S_L3=[0;66;0]/1000;
    S_L4v=[78.8;-19.5;-64.8]/1000;
    S_L4h=[78.8;19.5;-64.8]/1000;
    %% Rotation matrices
    R_01=rotx(pi/2)*roty(-pi/2)*rotz(-q1);
    %R_01=rotx(pi/2)*roty(pi)*rotz(q1);
    R_12=roty(-pi/2)*rotz(q2);
    R_23=roty(pi/2)*rotz(q3);
    R_34=rotx(-pi/2)*rotz(q4);
    %R_34=roty(-pi/2)*rotz(pi/2)*rotz(q4);
   % R_45v=rotx(pi)*rotz(q5v);
   % R_45h=rotx(pi)*rotz(q5h);
   % 
    %% Rotation matrices
    R_02 = R_01*R_12;
    R_03 = R_02*R_23;
    R_04 = R_03*R_34;
    %R_05v = R_04 * R_45v;
   % R_05h = R_04 * R_45h;

    %% Distance to center of mass from global frame
    R_c1=R_01*S_lc1;
    R_c2=R_01*S_L1+R_02*S_lc2;
    R_c3=R_01*S_L1+R_02*S_L2+R_03*S_lc3;
    R_c4=R_01*S_L1+R_02*S_L2+R_03*S_L3+R_04*S_lc4;
    %R_c5v=R_01*S_L1+R_02*S_L2+R_03*S_L3+R_04*S_L4v+R_05v*S_lc5v;
    %R_c5h=R_01*S_L1+R_02*S_L2+R_03*S_L3+R_04*S_L4h+R_05h*S_lc5h;
    %% Inertia in local frame 3x3 frame ( grams *  square millimeters ) - taken at the joint coordinate system
    
    
    I1_L = [153821.49   ,14337.86     ,-193910.07 ;
            14337.86      ,1040837.02 ,-795.59      ;     
            -193910.07  ,-795.59       ,960150.78  ;
            ];
        
    I2_L = [3827081.22	14382.51	-4248.42;
            14382.51	3889691.13	-801.19;
            -4248.42	 -801.19	155835.85;
            ];
    I3_L = [381100.17	-61206.22	0.85;
            -61206.22	47141.04	-2914.5;
            0.85	-2914.50	390294.02;
            ];

    
    I4_L = [753202.04	0.05	-664242.78;
            -0.05	 1406383.14	0.06;
            -664474.24	0.06    786380.38;
            ];
        
    I5v_L = [8908.43	3458.76	0.00;
            3458.76	121964.89	0.00;
            0.00	0.00	119613.32;
            ];
        
    I5h_L = [8908.43	-3458.76	0.00;
            -3458.76	121964.89	0.00;
            0.00	0.00	119613.32;
            ];
    

    % Convert to kg * square meters
    c = 10^-9; % ratio from grams per square millimeter to kg per square meters
    I1_L=I1_L*c;
    I2_L=I2_L*c;
    I3_L=I3_L*c;
    I4_L=I4_L*c;
    %I5v_L=I5v_L*c;
   % I5h_L=I5h_L*c;
    
    %% Inertia in global frame  
    %I1=R_01*I1_L;
    %I2=R_02*I2_L;
    %I3=R_03*I3_L;
    %I4=R_04*I4_L;
    %I5v=R_05v*I5v_L;
    %I5h=R_05h*I5h_L;

    
    I1=simplify(R_01*I1_L*R_01');
    I2=simplify(R_02*I2_L*R_02');
    I3=simplify(R_03*I3_L*R_03');
    I4=simplify(R_04*I4_L*R_04');
    %I5v=simplify(R_05v*I5v_L*R_05v');
    %I5h=simplify(R_05h*I5h_L*R_05h');
    
    
    m = [189.97;258.24;109.23;188.41;33.39;33.39];
    m = m*(1/1000);
    
    %% angular velocity
    z=[0;0;1];
    omega1 = dq1*(R_01)*z;
    omega2 = omega1+dq2*R_02*z;
    omega3 = omega2+dq3*R_03*z;
    omega4 = omega3+dq4*R_04*z;
    %omega5v = omega4+dq5v*R_05v*z;
    %omega5h = omega4+dq5h*R_05h*z;
    
    
    %linear velocity
    v1 = 0;
    v2 = v1 + cross(omega1,(R_01*S_L1));
    v3 = v2 + cross(omega2,(R_02*S_L2));
    v4 = v3 + cross(omega3,(R_03*S_L3));
   % v5v = v4 + cross(omega5v,(R_05v*S_L1));
  %  v5h = v4 + cross(omega5h,(R_05h*S_L1));
    
        %% linear velocity in center of mass
    vc1 = cross(omega1,(R_01*S_lc1));
    vc2 = v2 + cross(omega2,(R_02*S_lc2));
    vc3 = v3 + cross(omega3,(R_03*S_lc3));
    vc4 = v4 + cross(omega4,(R_04*S_lc4));
   % vc5v = v4 + cross(omega5v,(R_05v*S_lc5v));
    %vc5h = v4 + cross(omega5h,(R_05h*S_lc5h));
    %% Lagrange 
    syms g
    assume(g,'real')
    unit=[0;0;g]; 
    T1=1/2*omega1'*(I1*omega1);
    V1=-m1*R_c1'*unit;
    
    T2=1/2*m2*(vc2'*vc2)+1/2*omega2'*(I2*omega2);
    V2=-m2*R_c2'*unit;
    
    T3=1/2*m3*(vc3'*vc3)+1/2*omega3'*(I3*omega3);
    V3=-m3*R_c3'*unit;
    
    T4=1/2*m4*(vc4'*vc4)+1/2*omega4'*(I4*omega4);
    V4=-m4*R_c4'*unit;
    
    %T5v=1/2*m5*(vc5v'*vc5v)+1/2*omega5v'*(I1*omega5v);
    %V5v=m5*R_c5v'*unit;
    
    %T5h=1/2*m5*(vc5h'*vc5h)+1/2*omega5h'*(I1*omega5h);
    %V5h=m5*R_c5h'*unit;
   
    

    L=T1-V1+T2-V2+T3-V3+T4-V4;%+T5v-V5v+T5h-V5h;

    
    %diary on
    syms x
    firstTerm = [x;x;x;x;x;x];
    secondTerm = [x;x;x;x;x;x]; 
    tau = [x;x;x;x;x;x];
    G = [x;x;x;x;x;x];
    V = [x;x;x;x;x;x];
    M = sym('x', [6 6]);
    for index = 1:size(firstTerm)
        firstTerm(index) = diff(L,dq(index));
       
        firstTerm(index) = diff(firstTerm(index),t);
        
        for i = 1:size(firstTerm)
            firstTerm(index) = subs(firstTerm(index),diff(q(i), t),dq(i));
            firstTerm(index) = subs(firstTerm(index),diff(dq(i), t),ddq(i));
        end
        firstTerm(index);
        secondTerm(index) = diff(L,q(index));
        tau(index) = firstTerm(index)-secondTerm(index);
        
        %G(index) = collect(tau(index),g);
        %V(index) = collect(tau(index),[dq1 dq2 dq3 dq4 dq5v dq5h]);
        %for i = 1:6
        %    M(index,i) = collect(tau(index),ddq(i));
        %    for ii = 1:6
        %        M(index,i) = subs(M(index,i),ddq(ii),1);
        %    end
        %    
        % end
    end

end

