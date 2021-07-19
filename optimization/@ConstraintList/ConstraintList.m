classdef ConstraintList

    properties (SetAccess=public , GetAccess=public)
    
        FinalTime
        
        StepLength
        
        StepWidth
        
        StepHeight
        
        ForwardWalkingSpeed
      
        PhaseVariableDerivative
        
        SlewRate
        
        Trajectory
        
        COMPosition
        
        COMVelocity
        
        InitialPositions % convenient for swing up pendulum
        
        FinalPositions % convenient for swing up pendulum

        InitialVelocities % convenient for swing up pendulum
        
        FinalVelocities % convenient for swing up pendulum
        
        SwingFootHeight
        
        GroundClearance
        
        StanceFootInitialPosition
        
        StanceFootInitialVelocity
        
        SecondStanceFootInitialVelocity
        
        TorsoAngles
        
        OneStepPeriodic
        
        SwingFootVerticalImpactVelocity
        
        SwingFootForwardImpactVelocity
        
        SwingFootLateralImpactVelocity
        
        FootInterference
        
        %add by DAICHI
        GroundDefomationStanceLeg
        
        GroundDefomationPreSwingLeg
        
        SwingFootInitialPosition
    end
    
   
 
    
    methods
        
        function obj = ConstraintList()
            
        fdnames = fieldnames(obj);
        for i = 1:numel(fdnames)
            
            obj.(fdnames{i}) = Constraint();
            obj.(fdnames{i}).Bool = false;
            

        end
        

            
        end
        
    end
    

    
end

