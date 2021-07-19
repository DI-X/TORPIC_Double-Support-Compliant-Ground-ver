function [obj] = ConfigFunctions(obj, rbm, Kp, Kd, ContactModel)

arguments
    obj (1,1) NLP
    rbm (1,1) DynamicalSystem
    Kp (2,1) double
    Kd (2,1) double
    ContactModel (1,1) string
end

import casadi.*

% 2nd-order dynamics
[obj.Functions.DynamicsODESingle,obj.Functions.DynamicsODEDouble,...
    obj.Functions.GRFyStance,obj.Functions.GRFyPreSwing,...
    obj.Functions.GRFxStance,obj.Functions.GRFxPreSwing] = ContinuousSecondOrder(obj, rbm, Kp, Kd, ContactModel);

% Constrained 2nd-order dynamics
nContact = size(rbm.Contacts, 2);
if nContact ~= 0
    %[obj.Functions.ConstrainedDynamicsODESingle,obj.Functions.ConstrainedDynamicsODEDouble] = ConstrainedDynamics(obj, rbm);
end

% inertia matrix needed for impact
obj.Functions.InertiaMatrix = Function('f', {rbm.States.q.sym} , {rbm.Dynamics.H_matrix} );


% contact jacobian for impact
if nContact ~= 0
    for i = 1:1%nContact
        
        obj.Functions.ContactJacobian{i} = Function('f', {rbm.States.q.sym}, {rbm.Contacts{i}.Jac_contact});

    end
end

% position and velocity of center of mass        
obj.Functions.PositionCOM = Function('f', {rbm.States.q.sym} , {rbm.Dynamics.p_com});
obj.Functions.VelocityCOM = Function('f', {rbm.States.q.sym, rbm.States.dq.sym} , {rbm.Dynamics.v_com});


end