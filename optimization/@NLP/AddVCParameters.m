function [nlp , grid_var] = AddVCParameters(nlp, rbm, grid_var)


arguments
    nlp (1,1) NLP
    rbm (1,1) DynamicalSystem
    grid_var (1,1) struct
end



% idx is the number of the collocation point
%   e.g. idx = 0 means t = 0


if nlp.Problem.Trajectory.Bool
    
    switch nlp.Problem.Trajectory.PolyType
        case 'Bezier'            
            %for single
            a_ub = nlp.Problem.Trajectory.PolyCoeffS.UpperBound;
            a_lb = nlp.Problem.Trajectory.PolyCoeffS.LowerBound;
            a_guess = nlp.Problem.Trajectory.PolyCoeffS.Seed;
            
            n_coeff = numel(nlp.Problem.Trajectory.PolyCoeffS.Single.sym);
            
            [nlp, aS] = AddVar(nlp, 'aS_', n_coeff, reshape(a_guess, n_coeff, 1), reshape(a_lb, n_coeff, 1), reshape(a_ub, n_coeff, 1), 'aS');
            grid_var.aS = aS;
            
            
            %for double
            a_ub = nlp.Problem.Trajectory.PolyCoeffD.UpperBound;
            a_lb = nlp.Problem.Trajectory.PolyCoeffD.LowerBound;
            a_guess = nlp.Problem.Trajectory.PolyCoeffD.Seed;
            
            n_coeff = numel(nlp.Problem.Trajectory.PolyCoeffD.Double.sym);
            
            [nlp, aD] = AddVar(nlp, 'aD_', n_coeff, reshape(a_guess, n_coeff, 1), reshape(a_lb, n_coeff, 1), reshape(a_ub, n_coeff, 1), 'aD');
            grid_var.aD = aD;
            
        otherwise
            error('MyComponent:incorrectType',...
                'Error. \nVC type must be ''Bezier'', not %s.', nlp.Problem.Trajectory.PolyType)
            
    end
    
    
end




end




