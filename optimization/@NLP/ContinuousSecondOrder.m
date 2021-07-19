function [odefunSingle,odefunDouble,GRFyStance,GRFyPreSwing,GRFxStance,GRFxPreSwing,rbm]= ContinuousSecondOrder(nlp, rbm, Kp, Kd, ContactModel)

arguments
    nlp (1,1) NLP
    rbm (1,1) DynamicalSystem
    Kp (2,1) double
    Kd (2,1) double
    ContactModel (1,1) string
end

import casadi.*

% number of DOF
NB = rbm.Model.nd;

% states
q   = rbm.States.q.sym;
qd  = rbm.States.dq.sym;
qdd = rbm.States.ddq.sym;


% define control as decision variables
uS = rbm.Inputs.uS.sym;
uD = rbm.Inputs.uD.sym;

% inertia matrix
H = rbm.Dynamics.H_matrix;

% vector of coriolis and gravitational terms
C = rbm.Dynamics.C_terms;

% input matrix
BD = rbm.InputMapD;
BS = rbm.InputMapS;

Bsp=[0 0 0 0;
    0 0 0 0; 
    1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1;];

f1=0.31;
f2=0.06;
thetaTDKpreSwing=SX.sym('thetaTDKpreSwing');
thetaTDKStance=SX.sym('thetaTDKStance');

initialStancePos=SX.sym('initialStancePos');
initialPreSwingPos=SX.sym('initialStancePos');

ub=[f1*qd(3);f2*qd(4);f1*qd(5);f2*qd(6)]; %damping
usp1=-56.67*max(0,q(6)-thetaTDKStance);   %stance leg
usp2=-56.67*max(0,q(4)-thetaTDKpreSwing); %preswing leg

uspring1=[0;0;0;usp1]; %stance 
uspring2=[0;usp2;0;0]; %preswing

%usp=-ub+uspring;

Jc_1 = rbm.Contacts{1}.Jac_contact;
Jc_2 = rbm.Contacts{2}.Jac_contact;

variablesForGRFy={q,qd};
variablesForGRFxSt={q,qd,initialStancePos};
variablesForGRFxSw={q,qd,initialPreSwingPos};

nContact = size(rbm.Contacts, 2);

if nContact == 2
    
        %single support phase %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
        footzPositionStance=rbm.Contacts{1}.ContactFrame{2};
        footxPositionStance=rbm.Contacts{1}.ContactFrame{1};
        
        footVelStance=Jc_1*qd;
        
        yg1=footzPositionStance; %deformation is positive toward to the ground
        xg1=footxPositionStance-initialStancePos;
        
        if strcmp(ContactModel,'Spring-Damper')
            FStance=[-Kp(1)*xg1-Kd(1)*footVelStance(1);
                     -Kp(2)*yg1-Kd(2)*footVelStance(2);];
        elseif strcmp(ContactModel,'Nonlinear')
            error('Currently Nonlinear model is not supported')
        elseif strcmp(ContactModel,'Rigid')
            error('Currently Rigid model is not supported')
        end
             
        second_order_ode = H*qdd + C - BS*uS-Bsp*uspring1- Jc_1'*FStance;
        variables = {q,qd,qdd,uS,thetaTDKStance,initialStancePos};
    
        GRFyStance= Function('f', variablesForGRFy, {FStance(2)} );  
        GRFxStance= Function('f', variablesForGRFxSt, {FStance(1)} ); 
        
        odefunSingle = Function('f', variables, {second_order_ode} );    
        
        %double support phase %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        footzPositionPreSwing=rbm.Contacts{2}.ContactFrame{2};
        footxPositionPreSwing=rbm.Contacts{2}.ContactFrame{1};
        footVelPreSwing=Jc_2*qd;
        
        yg2=footzPositionPreSwing; %deformation is positive toward to the ground
        xg2=footxPositionPreSwing-initialPreSwingPos;
       
   
        if strcmp(ContactModel,'Spring-Damper')
            FPreSwing=[-Kp(1)*xg2-Kd(1)*footVelPreSwing(1);
                       -Kp(2)*yg2-Kd(2)*footVelPreSwing(2);];
        elseif strcmp(ContactModel,'Nonlinear')
            error('Currently Nonlinear model is not supported')
        elseif strcmp(ContactModel,'Rigid')
            error('Currently Rigid model is not supported')
        end       
               
        second_order_ode = H*qdd + C - BD*uD-Bsp*uspring1- Jc_1'*FStance- Jc_2'*FPreSwing- Bsp*uspring2;
        variables = {q,qd,qdd,uD,thetaTDKStance,thetaTDKpreSwing,initialStancePos,initialPreSwingPos};
  
        GRFyPreSwing= Function('f', variablesForGRFy, {FPreSwing(2)} );  
        GRFxPreSwing= Function('f', variablesForGRFxSw, {FPreSwing(1)} ); 
        odefunDouble = Function('f', variables, {second_order_ode} );

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

elseif nContact==1
        for i = 1:nContact

        Jc_i = rbm.Contacts{i}.Jac_contact;
        Fc_i = rbm.Contacts{i}.Fc.sym;

        second_order_ode = second_order_ode - Jc_i'*Fc_i;
        
        variables{end+1} = rbm.Contacts{i}.Fc.sym;

        end
        
        odefunSingle = Function('f', variables, {second_order_ode} );
        odefunDouble=0;
end




    
    
end