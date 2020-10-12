clc; clearvars; close all;

%% Definição do estados, variáveis de controle e de saida
% vetor de estados
x_vec = [ falcon.State('x',   0,  11000, 1e-4);...
          falcon.State('v',   0,  12.5, 1e-1) ];
% variavel de controle
u_vec = falcon.Control('i', 0,  63, 1);
% tempo final
tf = falcon.Parameter('FinalTime', 1400, 0, 1400, 1e-3);
% raio do cilindro no motor
rm = falcon.Parameter('raioM', 0.026, 0.020, 0.050, 1e2);
% variaveis de saida 
modeloutput = [...
    falcon.Output('Ft');...
    falcon.Output('Fa');...
    falcon.Output('Fr');...
    falcon.Output('Fg');...
    falcon.Output('theta')];

%% Construção do modelo, da restrição de caminho e o custo de Lagrange
% modelo do veiculo e pista
mdl = falcon.SimulationModelBuilder('DT1', x_vec, u_vec, rm);
mdl.addSubsystem(@mdl_dt1_pista, {'x', 'v', 'i', 'raioM'}, {'mdl_out'});               
mdl.SplitVariable('mdl_out', {'x_dot','v_dot', 'Ft', 'Fa', 'Fr', 'Fg', 'theta'}.')
mdl.setStateDerivativeNames('x_dot','v_dot');
mdl.setOutputs(modeloutput);
mdl.Build;
% restricao de caminho
lgc = falcon.PathConstraintBuilder('LCost', [], x_vec(2), u_vec(1), rm, @lagrange_cost);
lgc.Build;
% custo de lagrange
pti = falcon.PathConstraintBuilder('LimCor', [], x_vec(2), u_vec(1), rm, @i_max);
pti.Build;

%% Definição do problema de controle ótimo
% instancia do problema
problem = falcon.Problem('DT1_B1');
% discretização
tau = linspace(0,1,1001);
% fase 1 de 1
phase = problem.addNewPhase(@DT1, x_vec, tau, 0, tf);
phase.Model.setModelParameters(rm);
% malha do vetor de controle
phase.addNewControlGrid(u_vec, tau); 
% variaveis de saida do modelo
phase.Model.setModelOutputs(modeloutput);
% condições inciais e finais dos estados
phase.setInitialBoundaries([0;0]);
phase.setFinalBoundaries([10080;0],[10080;12.5]);
% restrição de corrente
pathc = phase.addNewPathConstraint(@LimCor,falcon.Constraint('Iup', -inf, 0, 1),tau);
pathc.setParameters(rm);
% funcional de custo
lagc = phase.addNewLagrangeCost(@LCost,falcon.Cost('Cost',1e-4));
% parametro otimizavel
lagc.setParameters(rm);

%% Calculo da solução
problem.setMajorIterLimit(10000)
problem.Solve();

%% Plots 
mostar_resultados(problem,rm)
