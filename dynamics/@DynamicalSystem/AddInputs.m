function obj = AddInputs(obj)

control = struct();

%single
control.uS = obj.Model.uS;

control.uS.ID = 'input';

obj.Inputs = control;

obj.Model = rmfield(obj.Model, 'uS');

%double
control.uD = obj.Model.uD;

control.uD.ID = 'input';

obj.Inputs = control;

obj.Model = rmfield(obj.Model, 'uD');


end