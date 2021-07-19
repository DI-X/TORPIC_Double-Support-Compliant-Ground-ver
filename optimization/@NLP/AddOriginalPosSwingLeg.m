function [nlp, grid_var] = AddOriginalPosSwingLeg(nlp, rbm, grid_var)


arguments
    nlp (1,1) NLP
    rbm (1,1) DynamicalSystem
    grid_var (1,1) struct
end
    


% idx is the number of the collocation point
%   e.g. idx = 0 means t = 0

q_guess = rbm.InitialPosSwToe;
[nlp, q_idx]   = AddVar(nlp, ['OriPosSwToe']  , 1, q_guess  , -inf  , inf  , 'OriPosSwToe');

grid_var.(['OriPosSwToe']) = q_idx;







end
