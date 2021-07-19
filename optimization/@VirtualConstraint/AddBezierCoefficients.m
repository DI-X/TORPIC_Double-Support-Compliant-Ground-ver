function alpha = AddBezierCoefficients(VirtualCon, rbm)

arguments
    VirtualCon (1,1) VirtualConstraint
    rbm (1,1) DynamicalSystem
end


import casadi.* % CasADi 3.4.5

switch VirtualCon.GaitPhase
    case 'Double'
        % number of actuated DOF
        number_actuated_DOF = size(rbm.InputMapD,2);
    case 'Single'
        number_actuated_DOF = size(rbm.InputMapS,2);
end

% order of bezier polynomial
bezier_order = VirtualCon.PolyOrder;


switch VirtualCon.GaitPhase
    case 'Double'
        bezier_order =VirtualCon.PolyOrder.Double;
    case 'Single'
        bezier_order =VirtualCon.PolyOrder.Single;
end

   
alpha = [];
for i = 1:number_actuated_DOF

    a{i} = SX.sym(['a' num2str(i)],bezier_order+1);
        
    alpha = [ alpha ; a{i}' ];

    
end


end