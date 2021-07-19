
syms q2 q1 lcm2 L2 lcm1 L1 m1 m2 J1 J2 real;

q=[q1,q2];
Pcm2=[lcm2*cos(q2);
      lcm2*sin(q2);];
Pcm1=[L2*cos(q2)+lcm1*cos(q2+q1);
        L2*sin(q2)+lcm1*sin(q2+q1);];
    
 Pcm2_dot=jacobian(Pcm2,q);
 Pcm1_dot=jacobian(Pcm1,q);
 
 abs2=q2;
 abs1=q2+q1;
 
 abs2_dot=jacobian(abs2,q);
 abs1_dot=jacobian(abs1,q);
 
 D=m2*Pcm2_dot'*Pcm2_dot+m1*Pcm1_dot'*Pcm1_dot+J2*abs2_dot'*abs2_dot+J1*abs1_dot'*abs1_dot;