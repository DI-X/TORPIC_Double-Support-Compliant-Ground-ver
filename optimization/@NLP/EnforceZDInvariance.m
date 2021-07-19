function [obj] = EnforceZDInvariance(obj, rbm, grid_var)


%% Argument Validation
arguments
    obj (1,1) NLP
    rbm (1,1) DynamicalSystem
    grid_var (1,1) struct
end

if obj.Problem.Trajectory.Bool

    switch obj.Problem.Trajectory.PolyType
        case 'Bezier'

            aD = reshape(grid_var.aD, numel(rbm.Inputs.uD.sym), obj.Problem.Trajectory.PolyOrder.Double+1);
            aS = reshape(grid_var.aS, numel(rbm.Inputs.uS.sym), obj.Problem.Trajectory.PolyOrder.Single+1);
            
            % Enforce hybrid invariance
            [obj] = EnforceHZDInvariance(obj, rbm, grid_var);

            
            % can change that but ideally gaits are robut to gains
            %ctrl_gain = 10;
            ctrl_gain = obj.Problem.Trajectory.ControlGain;
            
            for i = 1:obj.Settings.ncp

                switch obj.Problem.Trajectory.PolyPhase
                    case 'state-based'
                        tau_i = grid_var.(['tau_phase_', num2str(i)]);
                        t_plus = grid_var.p_phase_plus;
                        t_minus = grid_var.p_phase_minus;

                    case 'time-based'
                        tau_i = grid_var.(['tau_time_', num2str(i)]);
%                         t_plus = grid_var.p_time_plus;
%                         t_minus = grid_var.p_time_minus;
                        
                        t_plus_Double = grid_var.p_time_plus_Double;
                        t_minus_Double = grid_var.p_time_minus_Double;
                        t_plus_Single = grid_var.p_time_plus_Single;
                        t_minus_Single = grid_var.p_time_minus_Single;
                end
                
                if i<=round(obj.Settings.ncp/2)
                    y_i = obj.Functions.Y_Double_Controller(grid_var.(['pos_', num2str(i)]), aD, tau_i, t_plus_Double, t_minus_Double);
                    dy_i = obj.Functions.DY_Double_Controller(grid_var.(['pos_', num2str(i)]), grid_var.(['vel_', num2str(i)]), aD, tau_i, t_plus_Double, t_minus_Double);
                    ddy_i = obj.Functions.DDY_Double_Controller(grid_var.(['pos_', num2str(i)]), grid_var.(['vel_', num2str(i)]), grid_var.(['acc_', num2str(i)]), aD, tau_i, t_plus_Double, t_minus_Double);
                     [obj] = AddConstraint(obj, ddy_i + ctrl_gain^2*y_i + 2*ctrl_gain*dy_i, -obj.Settings.ConstraintTolerance*ones(numel(rbm.Inputs.uD.sym),1), obj.Settings.ConstraintTolerance*ones(numel(rbm.Inputs.uD.sym),1), 'ZD invariance');

                else
                    y_i = obj.Functions.Y_Single_Controller(grid_var.(['pos_', num2str(i)]), aS, tau_i, t_plus_Single, t_minus_Single);
                    dy_i = obj.Functions.DY_Single_Controller(grid_var.(['pos_', num2str(i)]), grid_var.(['vel_', num2str(i)]), aS, tau_i, t_plus_Single, t_minus_Single);
                    ddy_i = obj.Functions.DDY_Single_Controller(grid_var.(['pos_', num2str(i)]), grid_var.(['vel_', num2str(i)]), grid_var.(['acc_', num2str(i)]), aS, tau_i, t_plus_Single, t_minus_Single);
                     [obj] = AddConstraint(obj, ddy_i + ctrl_gain^2*y_i + 2*ctrl_gain*dy_i, -obj.Settings.ConstraintTolerance*ones(numel(rbm.Inputs.uS.sym),1), obj.Settings.ConstraintTolerance*ones(numel(rbm.Inputs.uS.sym),1), 'ZD invariance');

                end
                
                % enforce ZD invariance
                

            end
            
            

        otherwise
            error('not currently supported')

    end

end



end
