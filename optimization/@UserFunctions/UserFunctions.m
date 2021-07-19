classdef UserFunctions 
    
    properties (SetAccess=public , GetAccess=public)
    
        DynamicsODESingle
        
        DynamicsODEDouble
        
        ConstrainedDynamicsODESingle
        
        ConstrainedDynamicsODEDouble
      
        InertiaMatrix
        
        ContactJacobian

        PositionCOM
        
        VelocityCOM
        
        Y_Controller
        
        DY_Controller
        
        DDY_Controller
        
        %add by DAICHI
        
        Y_Single_Controller
        
        DY_Single_Controller
        
        DDY_Single_Controller
        
        Y_Double_Controller
        
        DY_Double_Controller
        
        DDY_Double_Controller
        
        GRFyPreSwing
        
        GRFyStance
        
        GRFxPreSwing
        
        GRFxStance
        
    end
    

    
    methods
        

        function obj = UserFunctions()

            
            
        end
        
    end
    

    
end

