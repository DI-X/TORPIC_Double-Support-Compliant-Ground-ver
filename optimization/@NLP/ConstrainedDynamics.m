function [hol_funSingle,hol_funDouble] = ConstrainedDynamics(nlp, rbm)


arguments
    nlp (1,1) NLP
    rbm (1,1) DynamicalSystem
end


import casadi.*


% number of DOF
NB = rbm.Model.nd;

% states
q   = rbm.States.q.sym;
qd  = rbm.States.dq.sym;
qdd = rbm.States.ddq.sym;


nContact = size(rbm.Contacts, 2);
mustBePositive(nContact)

holonomic_constraints = q(1)*[];


if nContact==1
    i=1;
    Jc = rbm.Contacts{i}.Jac_contact;
    
    dJc = rbm.Contacts{i}.dJac_contact;
    
    holonomic_constraints = [holonomic_constraints; Jc*qdd + dJc*qd];
    
    hol_funSingle = Function('f', {q, qd, qdd}, {holonomic_constraints} );
    hol_funDouble=0;
    
elseif nContact==2
    i = 1;
    
    Jc = rbm.Contacts{i}.Jac_contact;
    
    dJc = rbm.Contacts{i}.dJac_contact;
    
    holonomic_constraints = [holonomic_constraints; Jc(1,:)*qdd + dJc(1,:)*qd];
        
    hol_funSingle = Function('f', {q, qd, qdd}, {holonomic_constraints} );
    
    %DOuble
    i = 2;
    
    Jc = rbm.Contacts{i}.Jac_contact;
    
    dJc = rbm.Contacts{i}.dJac_contact;
    
    holonomic_constraints = [holonomic_constraints; Jc(1,:)*qdd + dJc(1,:)*qd];
    
    hol_funDouble = Function('f', {q, qd, qdd}, {holonomic_constraints} );
end


end