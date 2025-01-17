% function [position2enforce,velocity2enforce] = PeriodicityConstraints(rbm, H, impulseContribution, x0, xEnd, xFlipped)
function [PosPeriodicity,VelPeriodicity] = PeriodicityConstraints(rbm, H, impulseContribution, x0, xEnd, xFlipped)


q0 = x0(1:6);
dq0 = x0(7:12);

qEnd = xEnd(1:6);
dqEnd = xEnd(7:12);

qFlipped = xFlipped(1:6);
dqFlipped = xFlipped(7:12);


% dxDiscreteMap
dxMap = H*(dq0 - dqFlipped) - (impulseContribution);

% xDiscreteMap
xMap = q0 - qFlipped;

% xhip
%xMap(1) = 1;

% yhip
%xMap(2) = q0(2) - qEnd(2);


% x0 are the initial conditions
% xPlus are the values after impact

position2enforce = 2:6;
velocity2enforce = 1:6;


%R_map = Model.RelabelingMatrix();

%qm  = nlp.Functions.q_minus( x );
%dqm = nlp.Functions.qd_minus( x , dx );


%qm = R_map*x;
%dqm = R_map*dx;

PosPeriodicity = xMap(position2enforce);
VelPeriodicity = dxMap(velocity2enforce);


%[position2enforce,velocity2enforce] = Opt.PeriodicityConstraints(rbm, [xn; dxn], [x; dx], [qm; dqm]);


%nlp = AddConstraint(nlp, dxMap(velocity2enforce), -nlp.Settings.ConstraintTolerance*ones(numel(velocity2enforce),1), nlp.Settings.ConstraintTolerance*ones(numel(velocity2enforce),1), 'Periodicity (velocities)');


%nlp = AddConstraint(nlp, xMap(position2enforce), -nlp.Settings.ConstraintTolerance*ones(numel(position2enforce),1), nlp.Settings.ConstraintTolerance*ones(numel(position2enforce),1), 'Periodicity (positions)');     

    
    
end
