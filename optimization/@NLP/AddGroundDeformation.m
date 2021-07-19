function nlp = AddGroundDeformation(nlp, rbm, grid_var)

arguments
    nlp (1,1) NLP
    rbm (1,1) DynamicalSystem
    grid_var (1,1) struct
end

if nlp.Problem.GroundDefomationStanceLeg.Bool
    for idx = 1:nlp.Settings.ncp
        variables = grid_var.(['pos_', num2str(idx)]);
        [nlp] = AddConstraint(nlp, nlp.Problem.GroundDefomationStanceLeg.Function(variables), nlp.Problem.GroundDefomationStanceLeg.LowerBound, nlp.Problem.GroundDefomationStanceLeg.UpperBound, nlp.Problem.GroundDefomationStanceLeg.Name);
    end
end

if nlp.Problem.GroundDefomationPreSwingLeg.Bool
    for idx = 1:round(nlp.Settings.ncp/2)
        variables = grid_var.(['pos_', num2str(idx)]);
        [nlp] = AddConstraint(nlp, nlp.Problem.GroundDefomationPreSwingLeg.Function(variables), nlp.Problem.GroundDefomationPreSwingLeg.LowerBound, nlp.Problem.GroundDefomationPreSwingLeg.UpperBound, nlp.Problem.GroundDefomationPreSwingLeg.Name);
    end
end

end




