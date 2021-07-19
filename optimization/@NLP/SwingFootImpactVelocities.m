function [nlp] = SwingFootImpactVelocities(nlp, rbm, grid_var)

%% Argument Validation
arguments
    nlp (1,1) NLP
    rbm (1,1) DynamicalSystem
    grid_var (1,1) struct
end


if nlp.Problem.SwingFootVerticalImpactVelocity.Bool       
    [nlp] = AddConstraint(nlp, nlp.Problem.SwingFootVerticalImpactVelocity.Function(grid_var.(['pos_', num2str(nlp.Settings.ncp)]), grid_var.(['vel_', num2str(nlp.Settings.ncp)])), nlp.Problem.SwingFootVerticalImpactVelocity.LowerBound, nlp.Problem.SwingFootVerticalImpactVelocity.UpperBound, nlp.Problem.SwingFootVerticalImpactVelocity.Name);
end

if nlp.Problem.SwingFootLateralImpactVelocity.Bool
    [nlp] = AddConstraint(nlp, nlp.Problem.SwingFootLateralImpactVelocity.Function(grid_var.(['pos_', num2str(nlp.Settings.ncp)]), grid_var.(['vel_', num2str(nlp.Settings.ncp)])), nlp.Problem.SwingFootLateralImpactVelocity.LowerBound, nlp.Problem.SwingFootLateralImpactVelocity.UpperBound, nlp.Problem.SwingFootLateralImpactVelocity.Name);
end

if nlp.Problem.SwingFootForwardImpactVelocity.Bool
    for i=0:1%round(0.06*nlp.Settings.ncp)
     [nlp] = AddConstraint(nlp, nlp.Problem.SwingFootForwardImpactVelocity.Function(grid_var.(['pos_', num2str(nlp.Settings.ncp-i)]), grid_var.(['vel_', num2str(nlp.Settings.ncp-i)])), nlp.Problem.SwingFootForwardImpactVelocity.LowerBound, nlp.Problem.SwingFootForwardImpactVelocity.UpperBound, nlp.Problem.SwingFootForwardImpactVelocity.Name);
    end
    
    %preswing end of double vertical
    for i=0:round(0.03*nlp.Settings.ncp)
    % [nlp] = AddConstraint(nlp, nlp.Problem.SwingFootVerticalImpactVelocity.Function(grid_var.(['pos_', num2str(round(nlp.Settings.ncp/2)-i)]), grid_var.(['vel_', num2str(round(nlp.Settings.ncp/2)-i)])), 0, 0.001, nlp.Problem.SwingFootForwardImpactVelocity.Name);
    end
    
    %preswing end of double horizontal
    for i=0:round(0.03*nlp.Settings.ncp)
    % [nlp] = AddConstraint(nlp, nlp.Problem.SwingFootForwardImpactVelocity.Function(grid_var.(['pos_', num2str(round(nlp.Settings.ncp/2)-i)]), grid_var.(['vel_', num2str(round(nlp.Settings.ncp/2)-i)])), 0.01, inf, nlp.Problem.SwingFootForwardImpactVelocity.Name);
    end
end


end
