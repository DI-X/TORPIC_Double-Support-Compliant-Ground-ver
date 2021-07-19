function [obj] = EnforceHZDInvariance(obj, rbm, grid_var)

%% Argument Validation

% declare specific restrictions on function input arguments
arguments
    obj (1,1) NLP
    rbm (1,1) DynamicalSystem
    grid_var (1,1) struct
end


% number of actuated DOF
ND = numel(rbm.Inputs.uD.sym);
NS = numel(rbm.Inputs.uS.sym);

aD = grid_var.aD;
aS = grid_var.aS;

q_0_Double  = grid_var.pos_1;
qd_0_Double = grid_var.vel_1;

endOfDouble=round(obj.Settings.ncp/2);

q_0_Single= grid_var.(['pos_', num2str(endOfDouble+1)]);
qd_0_Single= grid_var.(['vel_', num2str(endOfDouble+1)]);

% position
[obj] = AddConstraint(obj, rbm.Model.H0D*q_0_Double - aD(1:ND), -obj.Settings.ConstraintTolerance*ones(ND,1), obj.Settings.ConstraintTolerance*ones(ND,1), 'HZD Invariance');
[obj] = AddConstraint(obj, rbm.Model.H0S*q_0_Single - aS(1:NS), -obj.Settings.ConstraintTolerance*ones(NS,1), obj.Settings.ConstraintTolerance*ones(NS,1), 'HZD Invariance');

% velocity        
switch obj.Problem.Trajectory.PolyPhase
    case 'time-based'
        t_plus_Double = grid_var.p_time_plus_Double;
        t_minus_Double = grid_var.p_time_minus_Double;
        t_plus_Single = grid_var.p_time_plus_Single;
        t_minus_Single = grid_var.p_time_minus_Single;
        [obj] = AddConstraint(obj, rbm.Model.H0D*qd_0_Double - (aD(ND+1:2*ND)-aD(1:ND))*obj.Problem.Trajectory.PolyOrder.Double/(t_minus_Double-t_plus_Double), -obj.Settings.ConstraintTolerance*ones(ND,1), obj.Settings.ConstraintTolerance*ones(ND,1), 'HZD Invariance');
        [obj] = AddConstraint(obj, rbm.Model.H0S*qd_0_Single - (aS(NS+1:2*NS)-aS(1:NS))*obj.Problem.Trajectory.PolyOrder.Single/(t_minus_Single-t_plus_Single), -obj.Settings.ConstraintTolerance*ones(NS,1), obj.Settings.ConstraintTolerance*ones(NS,1), 'HZD Invariance');

    case 'state-based'
        t_plus = grid_var.p_phase_plus;
        t_minus = grid_var.p_phase_minus;        
        [obj] = AddConstraint(obj, rbm.Model.H0D*qd_0 - (a(NA+1:2*NA)-a(1:NA))*obj.Problem.Trajectory.PolyOrder.Double*(rbm.Model.c*qd_0)/(t_minus-t_plus), -obj.Settings.ConstraintTolerance*ones(NA,1), obj.Settings.ConstraintTolerance*ones(NA,1), 'HZD Invariance');  

end





end
