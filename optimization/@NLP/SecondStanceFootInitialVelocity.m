function [nlp] = SecondStanceFootInitialVelocity(nlp, rbm, grid_var)

%% Argument Validation
arguments
    nlp (1,1) NLP
    rbm (1,1) DynamicalSystem
    grid_var (1,1) struct
end

if nlp.Problem.SecondStanceFootInitialVelocity.Bool
    [nlp] = AddConstraint(nlp, nlp.Problem.SecondStanceFootInitialVelocity.Function(grid_var.pos_1, grid_var.vel_1), nlp.Problem.SecondStanceFootInitialVelocity.LowerBound, nlp.Problem.SecondStanceFootInitialVelocity.UpperBound, nlp.Problem.SecondStanceFootInitialVelocity.Name);
end


% % foot interference
% if nlp.Problem.FootInterference.Bool
%     
%     for i = 1:nlp.Settings.ncp
%         [nlp] = AddConstraint(nlp, nlp.Problem.FootInterference.Function(grid_var.(['pos_', num2str(i)])), nlp.Problem.FootInterference.LowerBound, nlp.Problem.FootInterference.UpperBound, nlp.Problem.FootInterference.Name);
%     end
% 
% end

end
