function [nlp , grid_var] = AddFinalTime(nlp, rbm, grid_var)

arguments
    nlp (1,1) NLP
    rbm (1,1) DynamicalSystem
    grid_var (1,1) struct
end


tf_guess = nlp.Problem.FinalTime.Value;
tf_lb = nlp.Problem.FinalTime.LowerBound;
tf_ub = nlp.Problem.FinalTime.UpperBound;



% final time         
[nlp, tf] = AddVar(nlp, 'tf', 1, tf_guess, tf_lb, tf_ub, 'tf'); %end time of single = end of setp

[nlp, tf1] = AddVar(nlp, 'tf1', 1, tf_guess/10, tf_lb, tf_ub, 'tf1'); %end_time of double

grid_var.tf = tf;
grid_var.tf1 = tf1;
 

end




