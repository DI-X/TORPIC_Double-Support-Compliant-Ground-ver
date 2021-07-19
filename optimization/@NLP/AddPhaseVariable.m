function [obj, grid_var] = AddPhaseVariable(obj, rbm, grid_var)

%% Argument Validation
arguments
    obj (1,1) NLP
    rbm (1,1) DynamicalSystem
    grid_var (1,1) struct
end


%{
    Define 2 sets of phase variables (time and phase)
    Not as decision variables, but calculated variables
    Defining as decision variables should increase linearity but 
    p_var = c*q is very trivial
%}


nDoublePhase=round((obj.Settings.ncp)/2);
for i = 1:obj.Settings.ncp
    
    if i<=nDoublePhase
        tau_time_i = (i-1)/(nDoublePhase-1);
        grid_var.(['tau_time_', num2str(i)]) = tau_time_i;
    else
        tau_time_i = (i-1-nDoublePhase)/(obj.Settings.ncp-1-nDoublePhase);
        grid_var.(['tau_time_', num2str(i)]) = tau_time_i;
    end
end
grid_var.p_time_plus_Double = 0;            % beginning of phase
grid_var.p_time_minus_Double= grid_var.tf1; % end of phase
grid_var.p_time_plus_Single = grid_var.tf1; % beginning of phase
grid_var.p_time_minus_Single = grid_var.tf; % end of phase



p_var_0 = rbm.Model.c*grid_var.pos_1;
p_var_T = rbm.Model.c*grid_var.(['pos_', num2str(obj.Settings.ncp)]);
for i = 1:obj.Settings.ncp
    p_var_i = rbm.Model.c*grid_var.(['pos_', num2str(i)]);
    tau_phase_i = (p_var_i - p_var_0)/(p_var_T - p_var_0);
    grid_var.(['tau_phase_', num2str(i)]) = tau_phase_i;
end
grid_var.p_phase_plus = p_var_0; % beginning of phase
grid_var.p_phase_minus = p_var_T; % end of phase



% possible to define monotonically increasing quantity even if 
% time-based or open-loop are choosen 
if obj.Problem.PhaseVariableDerivative.Bool
    for i = 1:obj.Settings.ncp
        obj = AddConstraint(obj, rbm.Model.c*grid_var.(['vel_', num2str(i)]), obj.Problem.PhaseVariableDerivative.LowerBound, obj.Problem.PhaseVariableDerivative.UpperBound, 'Monotonic phase variable');     
    end
end




end
