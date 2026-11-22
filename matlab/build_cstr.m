function models = build_cstr
%BUILD_CSTR Part I, Question 3: plant, linearization and temperature tracking.
% Course rate constants use minutes; Simulink time and virtual input use seconds.
project = setup_project;load_system('simulink');
models = {'part1_cstr','part1_cstr_linearization','part1_cstr_tracking'};
p = struct('a',1,'beta',1,'H',200,'k0',7.2e10,'gamma',8750, ...
    'CAf',1,'Tf',350,'T0',310,'K',log(50)/5);
p.C0 = p.a*p.CAf/(p.a+p.k0*exp(-p.gamma/p.T0));
p.U0 = p.T0-(p.a*(p.Tf-p.T0)+p.H*p.k0*p.C0*exp(-p.gamma/p.T0))/p.beta;
for stage = 1:3
    model = models{stage};if bdIsLoaded(model),close_system(model,0);end
    new_system(model);workspace = get_param(model,'ModelWorkspace');names = fieldnames(p);
    for index = 1:numel(names),assignin(workspace,names{index},p.(names{index}));end
    set_param(model,'SolverType','Variable-step','Solver','ode45', ...
        'RelTol','1e-8','AbsTol','1e-10','MaxStep','0.02','StopTime','180', ...
        'ReturnWorkspaceOutputs','on','SaveOutput','off','SaveTime','off', ...
        'SignalLogging','off','AlgebraicLoopMsg','error','Location',[40 80 1380 850]);
    plant = [model '/Nonlinear CSTR'];
    if stage == 1
        createPlant(plant);
    else
        add_block([models{1} '/Nonlinear CSTR'],plant,'Position',[605 180 805 330]);
    end
    if stage == 1
        source = 'Coolant temperature';
        add_block('simulink/Sources/Step',[model '/' source],'Time','1', ...
            'Before','U0','After','U0+5','Position',[65 185 115 225]);
        wire(model,{[source '/1'],'Nonlinear CSTR/1'});
        command = [source '/1'];
    else
        controller = [model '/Linearizing controller'];
        add_block('built-in/Subsystem',controller,'Position',[330 180 520 330]);
        inputs = {'v','CA','T'};
        for index = 1:3
            add_block('simulink/Ports & Subsystems/In1',[controller '/' inputs{index}], ...
                'Port',num2str(index),'Position',[35 index*70 65 index*70+20]);
        end
        add_block('simulink/Signal Routing/Mux',[controller '/Arguments'], ...
            'Inputs','3','Position',[140 75 145 230]);
        add_block('simulink/User-Defined Functions/Fcn',[controller '/Coolant command'], ...
            'Expr','u(3)+(60*u(1)-a*(Tf-u(3))-H*k0*u(2)*exp(-gamma/u(3)))/beta', ...
            'Position',[240 115 750 175]);
        add_block('simulink/Ports & Subsystems/Out1',[controller '/Tc'], ...
            'Port','1','Position',[815 135 845 155]);
        for index = 1:3
            wire(controller,{[inputs{index} '/1'],sprintf('Arguments/%d',index)});
        end
        wire(controller,{'Arguments/1','Coolant command/1';'Coolant command/1','Tc/1'});
        wire(model,{'Nonlinear CSTR/2','Linearizing controller/2'; ...
            'Nonlinear CSTR/1','Linearizing controller/3';'Linearizing controller/1','Nonlinear CSTR/1'});
        command = 'Linearizing controller/1';
        if stage == 2
            source = 'Virtual temperature rate';
            add_block('simulink/Sources/Step',[model '/' source],'Time','1', ...
                'Before','0','After','0.1','Position',[65 185 115 225]);
            virtual = [source '/1'];
        else
            source = 'Temperature reference';
            add_block('simulink/Sources/Step',[model '/' source],'Time','1', ...
                'Before','310','After','330','Position',[65 110 115 150]);
            add_block('simulink/Math Operations/Sum',[model '/Temperature error'], ...
                'Inputs','+-','Position',[160 190 190 230]);
            add_block('simulink/Math Operations/Gain',[model '/Linear temperature control'], ...
                'Gain','K','NamePlacement','alternate','Position',[225 190 290 230]);
            wire(model,{'Temperature reference/1','Temperature error/1'; ...
                'Nonlinear CSTR/1','Temperature error/2'; ...
                'Temperature error/1','Linear temperature control/1'});
            virtual = 'Linear temperature control/1';
        end
        wire(model,{virtual,'Linearizing controller/1'});
    end
    scopes = {'Temperature scope','Concentration scope','Temperature rate scope','Coolant scope'};
    signals = {'T','CA','Tdot','Tc'};units = {'Temperature [K]','Concentration [mol/L]','Temperature rate [K/s]','Coolant temperature [K]'};
    sources = {'Nonlinear CSTR/1','Nonlinear CSTR/2','Nonlinear CSTR/3',command};
    for index = 1:4
        y = 140+index*80;
        add_block('simulink/Sinks/Scope',[model '/' scopes{index}],'Position',[1030 y 1090 y+40]);
        configuration = get_param([model '/' scopes{index}],'ScopeConfiguration');
        configuration.ShowGrid = true;configuration.ShowLegend = true;
        configuration.TimeSpan = '180';configuration.YLabel = units{index};
        wire(model,{sources{index},[scopes{index} '/1']});
        nameLine(model,sources{index},signals{index});
    end
    if stage>1
        set_param([model '/Temperature rate scope'],'NumInputPorts','2');
        wire(model,{virtual,'Temperature rate scope/2'});nameLine(model,virtual,'v');
    end
    if stage==3
        set_param([model '/Temperature scope'],'NumInputPorts','2');
        wire(model,{'Temperature reference/1','Temperature scope/2'});
        nameLine(model,'Temperature reference/1','reference');
    end
    note = Simulink.Annotation(model,sprintf([ ...
        'Part I - Question 3: non-isothermal CSTR; simulation time in seconds\n' ...
        'Course minute-based state equations divided by 60. v is K/s; u = Tc is K.\n' ...
        'Initial equilibrium: T = 310 K, CA = 1/(1+k(310)).\n' ...
        'Tracking model: reference 310 to 330 K at 1 s; ideal unconstrained coolant.']));
    note.Position = [60 10 1100 100];
    save_system(model,fullfile(project.simulink,[model '.slx']));
    Simulink.exportToVersion(model,fullfile(project.simulink,[model '_R2021a.slx']),'R2021a');
end
end

function createPlant(plant)
add_block('built-in/Subsystem',plant,'Position',[605 180 805 330]);
add_block('simulink/Ports & Subsystems/In1',[plant '/Tc'],'Position',[35 260 65 280]);
add_block('simulink/Signal Routing/Mux',[plant '/State and coolant'], ...
    'Inputs','3','Position',[140 100 145 280]);
add_block('simulink/User-Defined Functions/Fcn',[plant '/Mass balance'], ...
    'Expr','(a*(CAf-u(1))-k0*u(1)*exp(-gamma/u(2)))/60','Position',[230 70 655 125]);
add_block('simulink/User-Defined Functions/Fcn',[plant '/Energy balance'], ...
    'Expr','(a*(Tf-u(2))+H*k0*u(1)*exp(-gamma/u(2))+beta*(u(3)-u(2)))/60', ...
    'Position',[230 245 795 300]);
add_block('simulink/Continuous/Integrator',[plant '/Concentration'], ...
    'InitialCondition','C0','Position',[850 75 890 115]);
add_block('simulink/Continuous/Integrator',[plant '/Temperature'], ...
    'InitialCondition','T0','Position',[850 250 890 290]);
inputs = {'Concentration/1','Temperature/1','Tc/1'};
for index = 1:3,wire(plant,{inputs{index},sprintf('State and coolant/%d',index)});end
wire(plant,{'State and coolant/1','Mass balance/1';'State and coolant/1','Energy balance/1'; ...
    'Mass balance/1','Concentration/1';'Energy balance/1','Temperature/1'});
names = {'T','CA','Tdot'};sources = {'Temperature/1','Concentration/1','Energy balance/1'};
for index = 1:3
    y = 65+index*100;
    add_block('simulink/Ports & Subsystems/Out1',[plant '/' names{index}], ...
        'Port',num2str(index),'Position',[1050 y 1080 y+20]);
    wire(plant,{sources{index},[names{index} '/1']});
end
end

function wire(system,connections)
for index = 1:size(connections,1)
    add_line(system,connections{index,1},connections{index,2},'autorouting','on');
end
end

function nameLine(model,source,name)
parts = strsplit(source,'/');ports = get_param([model '/' parts{1}],'PortHandles');
set_param(get_param(ports.Outport(str2double(parts{2})),'Line'),'Name',name);
end
