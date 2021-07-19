function [nlp] = EnforceDynamics(nlp, rbm, grid_var)


arguments
    nlp (1,1) NLP
    rbm (1,1) DynamicalSystem
    grid_var (1,1) struct
end


NB = numel(rbm.States.q.sym);


nContact = size(rbm.Contacts, 2);

% number of holonomic constraints
% number of holonomic constraints
%NHC = size(rbm.J_contact,1);

NH = 0;
for i = 1:nContact
    NH = NH + size(rbm.Contacts{i}.Jac_contact,1);
end

% idx is the number of the collocation point
%   e.g. idx = 0 means t = 0
for idx = 1:nlp.Settings.ncp
    
    if idx<=round(nlp.Settings.ncp/2) %double support phase
        
        vars_cp = {grid_var.(['pos_', num2str(idx)]),...
            grid_var.(['vel_', num2str(idx)]),...
            grid_var.(['acc_', num2str(idx)])};
        
        if nContact ~= 0
            NH=2; %two legs only for x direction
           % nlp = AddConstraint(nlp, nlp.Functions.ConstrainedDynamicsODEDouble(vars_cp{:}), -nlp.Settings.ConstraintTolerance*ones(NH,1), nlp.Settings.ConstraintTolerance*ones(NH,1), 'dynamics (HC)');
        end
        
        vars_cp{end+1} = grid_var.(['input_', num2str(idx)]);
        
        vars_cp{end+1} =grid_var.(['pos_', num2str(1)])(6,1);% stance knee touch down angle
        
        vars_cp{end+1} =grid_var.(['pos_', num2str(1)])(6,1); %preswing knee touch down angle        
             
        tempKK=nlp.Problem.StanceFootInitialPosition.Function(grid_var.pos_1);
        
        vars_cp{end+1} = tempKK(1);                             % initial stance x postion
        
        %vars_cp{end+1} = nlp.Problem.SwingFootInitialPosition.Function(grid_var.pos_1);  %initial Swing x postion
        vars_cp{end+1} = grid_var.(['OriPosSwToe']);
        
        % enforce equations of motion implicitly

        nlp = AddConstraint(nlp, nlp.Functions.DynamicsODEDouble(vars_cp{:}), -nlp.Settings.ConstraintTolerance*ones(NB,1), nlp.Settings.ConstraintTolerance*ones(NB,1), 'dynamics (EOM)');
        
    else
        NH=1;        %rewrite for single support for only x direction
        nContact=1;  %rewrite for single support
        
        vars_cp = {grid_var.(['pos_', num2str(idx)]),...
            grid_var.(['vel_', num2str(idx)]),...
            grid_var.(['acc_', num2str(idx)])};
        
        if nContact ~= 0
           % nlp = AddConstraint(nlp, nlp.Functions.ConstrainedDynamicsODESingle(vars_cp{:}), -nlp.Settings.ConstraintTolerance*ones(NH,1), nlp.Settings.ConstraintTolerance*ones(NH,1), 'dynamics (HC)');
        end
        
        vars_cp{end+1} = grid_var.(['input_', num2str(idx)]);
        
        vars_cp{end+1} = grid_var.(['pos_', num2str(1)])(6,1);% stance knee touch down angle
                    
        tempKK=nlp.Problem.StanceFootInitialPosition.Function(grid_var.pos_1);
        
        vars_cp{end+1} = tempKK(1);                           % initial stance x postion   
        
        % enforce equations of motion implicitly
        nlp = AddConstraint(nlp, nlp.Functions.DynamicsODESingle(vars_cp{:}), -nlp.Settings.ConstraintTolerance*ones(NB,1), nlp.Settings.ConstraintTolerance*ones(NB,1), 'dynamics (EOM)');
        

    end
end


end


