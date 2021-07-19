function [obj, rbm] = ConList(obj, rbm, ncp)
% this function loads the constraints

arguments
    obj (1,1) ConstraintList
    %traj (1,1) %double
    rbm (1,1) DynamicalSystem
    ncp (1,1) double
end

%fknfr
%validateattributes('ConstraintList')


import casadi.*

% final time (s)
obj.FinalTime.Name = 'Step period';
obj.FinalTime.Bool = false;
obj.FinalTime.LowerBound = 0.25;
obj.FinalTime.UpperBound = 0.75;


% step length (m)
obj.StepLength.Name = 'Step length';
obj.StepLength.Bool = true;
obj.StepLength.Function = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{4,2}(1)});
obj.StepLength.LowerBound = 0.15;
obj.StepLength.UpperBound = 0.4;


% step height (m)  % height of swing leg at the end of the step
obj.StepHeight.Name = 'Step height';      
obj.StepHeight.Bool = true;
obj.StepHeight.Function = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{4,2}(3)});
obj.StepHeight.LowerBound = 0;      
obj.StepHeight.UpperBound = 0;


% desired walking speed (m/s)
obj.ForwardWalkingSpeed.Name = 'Walking speed';      
obj.ForwardWalkingSpeed.Bool = false;
obj.ForwardWalkingSpeed.LowerBound = 0.30;      
obj.ForwardWalkingSpeed.UpperBound = 0.35;


% minimum velocity phase variable (rad/s or m/s)
obj.PhaseVariableDerivative.Name = 'Theta dot';
obj.PhaseVariableDerivative.Bool = true;
obj.PhaseVariableDerivative.LowerBound = 0.05;
obj.PhaseVariableDerivative.UpperBound = Inf;


% swing foot height (m)%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
obj.SwingFootHeight.Name = 'Swing foot height';
obj.SwingFootHeight.Bool = true;
%obj.SwingFootHeight.Timing = ceil(ncp/2);
obj.SwingFootHeight.Timing = ceil((ncp+round(ncp/2))/2);
obj.SwingFootHeight.Function = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{4,2}(3)});
obj.SwingFootHeight.LowerBound = 0.05;
obj.SwingFootHeight.UpperBound = Inf;


% ground clearance (m)%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
obj.GroundClearance.Name = 'Ground clearance';
obj.GroundClearance.Bool = true;
obj.GroundClearance.Function = Function('f', {rbm.States.q.sym}, {rbm.BodyPositions{4,2}(3)});
obj.GroundClearance.LowerBound = 0;
obj.GroundClearance.UpperBound = Inf;


% stance foot initial position (m)
obj.StanceFootInitialPosition.Name = 'Stance position at 0';
obj.StanceFootInitialPosition.Bool = true;
obj.StanceFootInitialPosition.Function = Function('f', {rbm.States.q.sym}, {[rbm.BodyPositions{6,2}(1); rbm.BodyPositions{6,2}(3)]});
obj.StanceFootInitialPosition.LowerBound = [0;0];
obj.StanceFootInitialPosition.UpperBound = [0;0];


% stance foot initial velocity (m/s)
obj.StanceFootInitialVelocity.Name = 'Stance velocity at 0';
obj.StanceFootInitialVelocity.Bool = true;
% obj.StanceFootInitialVelocity.Function = Function('f', {rbm.States.q.sym, rbm.States.dq.sym}, {[rbm.BodyVelocities{6,2}(1); rbm.BodyVelocities{6,2}(3)]});
% obj.StanceFootInitialVelocity.LowerBound = [0;0];
% obj.StanceFootInitialVelocity.UpperBound = [0;0];
obj.StanceFootInitialVelocity.Function = Function('f', {rbm.States.q.sym, rbm.States.dq.sym}, {[rbm.BodyVelocities{6,2}(1)]});
obj.StanceFootInitialVelocity.LowerBound = 0;
obj.StanceFootInitialVelocity.UpperBound = 0;

% Second stance foot (Swing foot) initial velocity (m/s)%%%%%%%%%%%%%%
obj.SecondStanceFootInitialVelocity.Name = 'Second Stance velocity at 0';
obj.SecondStanceFootInitialVelocity.Bool = true;
obj.SecondStanceFootInitialVelocity.Function = Function('f', {rbm.States.q.sym, rbm.States.dq.sym}, {[rbm.BodyVelocities{4,2}(1); rbm.BodyVelocities{4,2}(3)]});
obj.SecondStanceFootInitialVelocity.LowerBound = [0;0];
obj.SecondStanceFootInitialVelocity.UpperBound = [0;0];

%Second Stance foot initial X position %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
obj.SwingFootInitialPosition.Name = 'Swing foot position';
obj.SwingFootInitialPosition.Bool = false;
obj.SwingFootInitialPosition.Function = Function('f', {rbm.States.q.sym}, {[rbm.BodyPositions{4,2}(1)]});
obj.SwingFootInitialPosition.LowerBound = [0;0];
obj.SwingFootInitialPosition.UpperBound = [0;0];

% Ground Deformation for Stance leg    %%%%%%%%%%%%%%
obj.GroundDefomationStanceLeg.Name = 'Ground Deformation for Stance leg';
obj.GroundDefomationStanceLeg.Bool = true;
obj.GroundDefomationStanceLeg.Function = Function('f', {rbm.States.q.sym}, {rbm.Contacts{1}.ContactFrame{2}});
obj.GroundDefomationStanceLeg.LowerBound = -0.20;
obj.GroundDefomationStanceLeg.UpperBound = -0;

% Ground Deformation for PreSwing leg   %%%%%%%%%%%%%%
obj.GroundDefomationPreSwingLeg.Name = 'Ground Deformation for PreSwing Leg';
obj.GroundDefomationPreSwingLeg.Bool = true;
obj.GroundDefomationPreSwingLeg.Function = Function('f', {rbm.States.q.sym}, {rbm.Contacts{2}.ContactFrame{2}});
obj.GroundDefomationPreSwingLeg.LowerBound = -0.20;
obj.GroundDefomationPreSwingLeg.UpperBound = -0;

% swing foot vertical impact velocity (m/s) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
obj.SwingFootVerticalImpactVelocity.Name = 'Swing foot vertical impact velocity';
obj.SwingFootVerticalImpactVelocity.Bool = false;
obj.SwingFootVerticalImpactVelocity.Function = Function('f', {rbm.States.q.sym, rbm.States.dq.sym}, {rbm.BodyVelocities{4,2}(3)});
obj.SwingFootVerticalImpactVelocity.LowerBound = -0.66;
obj.SwingFootVerticalImpactVelocity.UpperBound = -0.35;


% swing foot forward impact velocity (m/s)%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
obj.SwingFootForwardImpactVelocity.Name = 'Swing foot forward impact velocity';
obj.SwingFootForwardImpactVelocity.Bool = true;
obj.SwingFootForwardImpactVelocity.Function = Function('f', {rbm.States.q.sym, rbm.States.dq.sym}, {rbm.BodyVelocities{4,2}(1)});
obj.SwingFootForwardImpactVelocity.LowerBound = -0.0001;
obj.SwingFootForwardImpactVelocity.UpperBound = 0.0001;


% slew rate (N.m/s) %never turn on for the gait that has both of double and
% single support phase
obj.SlewRate.Name = 'Actuator slew rate';
obj.SlewRate.Bool = false;
obj.SlewRate.LowerBound = -1/0.001;
obj.SlewRate.UpperBound = 1/0.001;


% Bezier coefficients
if obj.Trajectory.Bool
    obj.Trajectory.PolyCoeffS.LowerBound = -5*ones(4,6);
    obj.Trajectory.PolyCoeffS.UpperBound = 5*ones(4,6);
end

if obj.Trajectory.Bool
    obj.Trajectory.PolyCoeffD.LowerBound = -5*ones(2,6);
    obj.Trajectory.PolyCoeffD.UpperBound = 5*ones(2,6);
end

% periodicity constraints
obj.OneStepPeriodic.Name = 'Periodicity constraints';
obj.OneStepPeriodic.Bool = true;

% velocity bounds
rbm.States.dq.LowerBound = -20*ones(6,ncp);
rbm.States.dq.UpperBound = 20*ones(6,ncp);

% acceleration bounds
rbm.States.ddq.LowerBound = -100*ones(6,ncp);
rbm.States.ddq.UpperBound = 100*ones(6,ncp);

% torque limits (N.m)
torque_limits = 100; 
rbm.Inputs.uS.LowerBound = -torque_limits*ones(4,ncp-round(ncp/2));
rbm.Inputs.uS.UpperBound = torque_limits*ones(4,ncp-round(ncp/2));

rbm.Inputs.uD.LowerBound = -torque_limits*ones(2,round(ncp/2));
rbm.Inputs.uD.UpperBound = torque_limits*ones(2,round(ncp/2));

% contact forces (N)
warning('add new kind of contact for planar')

rbm.Contacts{1}.Fc.LowerBound = -1000*ones(1,ncp);
rbm.Contacts{1}.Fc.UpperBound = 1000*ones(1,ncp);

rbm.Contacts{2}.Fc.LowerBound = -1000*ones(1,round(ncp/2));
rbm.Contacts{2}.Fc.UpperBound = 1000*ones(1,round(ncp/2));

warning('add flexible constraints')

warning('add normal Force as flexible constraint')

% position bounds
rbm.States.q.LowerBound = -pi*ones(6,ncp);
rbm.States.q.UpperBound = pi*ones(6,ncp);

% x_hip
rbm.States.q.LowerBound(1,:) = -0.5*ones(1,ncp);
rbm.States.q.UpperBound(1,:) = 0.5*ones(1,ncp);

% z_hip
rbm.States.q.LowerBound(2,:) = 0.5*ones(1,ncp);
rbm.States.q.UpperBound(2,:) = 1.25*ones(1,ncp);

% torso
rbm.States.q.LowerBound(3,:) = -pi/6*ones(1,ncp);
rbm.States.q.UpperBound(3,:) = pi/6*ones(1,ncp);


% knees don't hyperextend
rbm.States.q.LowerBound(4,:) = deg2rad(5)*ones(1,ncp);
rbm.States.q.LowerBound(6,:) = deg2rad(5)*ones(1,ncp);



end