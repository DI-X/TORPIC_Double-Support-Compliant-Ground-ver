%% LOAD DYNAMICAL SYSTEM

clear all; clc; close all; %#ok<*CLALL>

[rbm] = ld_model(...
    {'model', @Model.Edited_planar_7_dof_biped},...
    {'debug', false});

%% SPECIFY CONTACT

% stance leg end
rbm.Contacts{1} = Contact(rbm, 'Point',...
    {'Friction', true},...
    {'FrictionCoefficient', 0.6},...
    {'FrictionType', 'Pyramid'},...
    {'ContactFrame', rbm.BodyPositions{6,2}([1,3])});

%swing leg during the double support phase
rbm.Contacts{2} = Contact(rbm, 'Point',...
    {'Friction', true},...
    {'FrictionCoefficient', 0.6},...
    {'FrictionType', 'Pyramid'},...
    {'ContactFrame', rbm.BodyPositions{4,2}([1,3])});

%% CREATE NLP

nlp = NLP(rbm,...
    {'NFE',25},...    % must be even number for contact force of swing leg that has it only in double support
    {'CollocationScheme', 'HermiteSimpson'},...
    {'LinearSolver', 'mumps'},...
    {'ConstraintTolerance', 1E-4});

% Create functions for dynamics equations 

Kp=[10000;10000]; %Ground stiffness in horizontal and vertical directions
Kd=[1250;1250]; %Ground damping in horizontal and vertical directions
nlp = ConfigFunctions(nlp, rbm, Kp, Kd, 'Spring-Damper');


%% Virtual constraint
nlp = AddVirtualConstraints(nlp, rbm,...
    {'PolyType', 'Bezier'},...
    {'PolyOrder', 5},...
    {'PolyPhase', 'time-based'},...
    {'GaitPhase', 'Double'});

nlp = AddVirtualConstraints(nlp, rbm,...
    {'PolyType', 'Bezier'},...
    {'PolyOrder', 5},...
    {'PolyPhase', 'time-based'},...
    {'GaitPhase', 'Single'});


% load user-defined constraints
[nlp, rbm] = LoadConstraints(nlp, rbm);


%% LOAD SEED

%[nlp, rbm] = LoadSeed(nlp, rbm);
[nlp, rbm] = LoadSeed(nlp, rbm, 'planar-7-dof-seed.mat');
 

%% FILL UP & SOLVE NLP

nlp = ParseNLP(nlp, rbm);
%parpool
nlp = SolveNLP(nlp);


%% EXTRACT SOLUTION

% structure containing the solution data
data = ExtractData(nlp, rbm);


%% ANIMATE SOLUTION

qAnim = data.pos;
tAnim = data.t;

% options: true or false 
anim_options.bool = true;

anim_options.axis.x = [-0.1 0.6];
anim_options.axis.y = [-0.1 0.4];
anim_options.axis.z = [0 1.3];

% skips frame to animate faster 
anim_options.skip_frame = 1;

anim_options.views = {'3D','sagittal'};

% save options
anim_options.save_movie    = true;
anim_options.movie_name    = 'planar_7dof_biped.mp4';
anim_options.movie_quality = 75; % scalar between [0 100], default 75
anim_options.movie_fps     = 30;  % frame rate, default 30

% create a light object or not
anim_options.lights = true;

% canss figure as 5th argument
Anim.animate(rbm, tAnim, qAnim, anim_options)
%set(gcf,'menubar','figure')
%set(gcf,'menubar','none') pa
%% ANIMATE SOLUTION

qAnim = data.pos;
tAnim = data.t;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% optimized a left step (right leg in stance)
qLeftStep = data.pos;

% Flip states
R = Model.RelabelingMatrix();

% compute symmetric right step 
qRightStep = R*data.pos;
qRightStep(1,:) = qRightStep(1,:) + range(qLeftStep(1,:));
qRightStep(2,:) = qRightStep(2,:) + data.pbody{6,1}(2,end) + data.pbody{4,1}(2,end);

% append
qAnim = [qAnim, qRightStep];
tAnim = [tAnim, data.t + tAnim(end)];

% total number of strides (2x steps)
N_strides = 2;
for i = 1:N_strides-1

    % left step
    qLeftStep = data.pos;
    qLeftStep(1,:) = qLeftStep(1,:) + range(qAnim(1,:));
    qAnim = [qAnim, qLeftStep]; %#ok<*AGROW>
    tAnim = [tAnim, data.t + tAnim(end)];

    % right step
    qRightStep = R*data.pos;
    qRightStep(1,:) = qRightStep(1,:) + range(qAnim(1,:));
    qRightStep(2,:) = qRightStep(2,:) + data.pbody{6,1}(2,end) + data.pbody{4,1}(2,end);
    qAnim = [qAnim, qRightStep];
    tAnim = [tAnim, data.t + tAnim(end)];

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% options: true or false 
anim_options.bool = true;

anim_options.axis.x = [-0.1 0.6];
anim_options.axis.y = [-0.1 0.4];
anim_options.axis.z = [0 1.3];

% skips frame to animate faster 
anim_options.skip_frame = 1;

% views to show. Options: {'3D','frontal','sagittal','top'}
%anim_options.views = {'3D','frontal','sagittal','top'};
anim_options.views = {'sagittal'};

% save options
anim_options.save_movie    = true;
anim_options.movie_name    = 'spatial_12_dof.mp4';
anim_options.movie_quality = 100; % scalar between [0 100], default 75
anim_options.movie_fps     = 30;  % frame rate, default 30

% create a light object or not
anim_options.lights = true;

% can pass figure as 5th argument
Anim.animate(rbm, tAnim, qAnim, anim_options)
%set(gcf,'menubar','figure')
%set(gcf,'menubar','none')



%% SAVE SEED

seed.q = data.pos;
seed.qd = data.vel;
seed.qdd = data.acc;
seed.t = data.t;
% seed.Fc_1 = data.Fc1;
% seed.Fc_2 = data.Fc2;
seed.aD = data.aD;
seed.aS = data.aS;
seed.uD = data.input_D;
seed.uS = data.input_S;
%seed.du = data.der_input;

% can be used as seed
str2save = 'planar-7-dof-seed.mat';
save(str2save, 'seed')
%% Increase the number collocation pointby interporating seeds
% 
%  seed=interpolateseeds(data);
% 
%  %%can be used as seed
%  str2save = 'planar-7-dof-seed.mat';
%  save(str2save, 'seed')
