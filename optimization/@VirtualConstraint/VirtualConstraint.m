classdef VirtualConstraint 
    
    properties (SetAccess=public , GetAccess=public)
    
        Bool
        
        PolyType
        
        PolyOrder
      
        PolyPhase
        
        GaitPhase
        
        PolyCoeff
        
        PolyCoeffS
        
        PolyCoeffD
        
        ControlGain
        
    end
    
   
    
    methods
        
        function obj = VirtualConstraint()
   
        
            obj.Bool = true;
            
            obj.ControlGain = 10;
            
        end
        
    end
    

    
end

