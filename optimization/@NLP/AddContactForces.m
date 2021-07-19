function [nlp , grid_var] = AddContactForces(nlp, rbm, grid_var)

arguments
    nlp (1,1) NLP
    rbm (1,1) DynamicalSystem
    grid_var (1,1) struct
end

NC = numel(rbm.Contacts);

for i = 1:NC
    % idx is the number of the collocation point
    %   e.g. idx = 0 means t = 0
%     NFc = numel(rbm.Contacts{i}.Fc.sym);
    
    for idx = 1:round(nlp.Settings.ncp/i)
        
%         Fc_ub = rbm.Contacts{i}.Fc.UpperBound(:, idx);
%         Fc_lb = rbm.Contacts{i}.Fc.LowerBound(:, idx);
%         Fc_guess = rbm.Contacts{i}.Fc.Seed(:, idx);
%         
%         [nlp, Fc_idx] = AddVar(nlp, ['Fc', num2str(i), '_', num2str(idx)], NFc, Fc_guess, Fc_lb, Fc_ub, ['Fc', num2str(i)]);
%         grid_var.(['Fc', num2str(i),'_', num2str(idx)]) = Fc_idx;
        
        if rbm.Contacts{i}.Friction.bool
            
            %[nlp] = EnforceFriction(nlp, rbm, rbm.Contacts{i}, grid_var.(['Fc', num2str(i),'_', num2str(idx)]));
            
            variables={grid_var.(['pos_', num2str(idx)]),...
                grid_var.(['vel_', num2str(idx)])};
            variablesXst={grid_var.(['pos_', num2str(idx)]),...
                grid_var.(['vel_', num2str(idx)])};
            variablesXsw={grid_var.(['pos_', num2str(idx)]),...
                grid_var.(['vel_', num2str(idx)])};
               
            variablesXsw{end+1} =grid_var.(['OriPosSwToe']);
            
            tempKK=nlp.Problem.StanceFootInitialPosition.Function(grid_var.pos_1);
            variablesXst{end+1}=tempKK(1);
            if i==1
                [nlp] = EnforceFrictionCompliantGR(nlp, rbm, rbm.Contacts{i}, nlp.Functions.GRFxStance(variablesXst{:}),nlp.Functions.GRFyStance(variables{:}));
            elseif i==2
                 [nlp] = EnforceFrictionCompliantGR(nlp, rbm, rbm.Contacts{i}, nlp.Functions.GRFxPreSwing(variablesXsw{:}),nlp.Functions.GRFyPreSwing(variables{:}));
                 if idx>=round(nlp.Settings.ncp/i)
                     [nlp] = forcelimit(nlp, rbm, rbm.Contacts{i}, nlp.Functions.GRFxPreSwing(variablesXsw{:}),nlp.Functions.GRFyPreSwing(variables{:}));
                 end
            end
        end
        
    end
    
end

end




