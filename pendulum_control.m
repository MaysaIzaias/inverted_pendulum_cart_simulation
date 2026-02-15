

% Carrinho equilibrista - VERSÃO CORRIGIDA
clear all, close all, clc;

%% Inicialização CoppeliaSim
try
    client = RemoteAPIClient();
    sim = client.require('sim');
    client.setStepping(true);
    jointR_handle = sim.getObject('/Revolute_joint_right');
    jointL_handle = sim.getObject('/Revolute_joint_left');
    carrinho_handle = sim.getObject('/CARRINHO_estrutura');
    disp('✅ Conectado ao CoppeliaSim');
catch ME
    error(['❌ Erro: ' ME.message]);
end

%% 🎯 PARÂMETROS CALIBRADOS
M = 0.5;      % massa do carrinho (kg)
m = 0.2;      % massa do pêndulo (kg)
l = 0.3;      % comprimento (m)
g = 9.81;     % gravidade
bx = 2.5;     % atrito viscoso
bo = 1.0;     % amortecimento angular
km = 0.16;    % constante do motor

%% Espaço de estados
A22 = -bx/M;
A23 = -(m*g)/M;
A24 = -(bo*M*l);
A42 = bx/M;
A43 = (g*(M+m))/(M*l);
A44 = -(bo*(M+m))/(M*m*l*l);
B2 = km/M;
B4 = -km/(M*l);

A = [0  1   0   0
     0 A22 A23 A24
     0  0   0   1
     0 A42 A43 A44];
B = [0; B2; 0; B4];

%% Controlador LQR
q1 = 100;  q2 = 10;  q3 = 80;  q4 = 1;  r = 0.5;
Q = diag([q1, q2, q3, q4]);
R = r;
K = lqr(A, B, Q, R);
fprintf('Ganhos LQR: K = [%.3f, %.3f, %.3f, %.3f]\n', K(1), K(2), K(3), K(4));

%% Configuração da simulação
dt = 0.05;
simulation_time = 20;
steps = simulation_time / dt;

% Arrays para logging
time_log = zeros(steps, 1);
position_log = zeros(steps, 1);
angle_log = zeros(steps, 1);
control_log = zeros(steps, 1);
velocity_log = zeros(steps, 1);
ang_velocity_log = zeros(steps, 1);
reference_log = zeros(steps, 1);

%% 🎯 CONFIGURAÇÕES CORRIGIDAS
velocity_gain = 0.05;  % AUMENTADO para movimento mais efetivo
posicao_desejada = 0;
fase_atual = 1;
tempo_inicio_fase = 0;

% Filtro menos agressivo
alpha = 0.3;

%% Loop principal - VERSÃO CORRIGIDA
disp('=== INICIANDO CONTROLE CORRIGIDO ===');
try
    sim.startSimulation();
    pause(2);  % Mais tempo para estabilização inicial
    
    % Obter posição inicial para calibração
    pos_data = sim.getObjectPosition(carrinho_handle, -1);
    if iscell(pos_data)
        pos_inicial = double(pos_data{1});
    else
        pos_inicial = double(pos_data(1));
    end
    
    fprintf('Posição inicial do carrinho: %.3f m\n', pos_inicial);
    
    last_pos = pos_inicial;
    last_angle = 0;
    last_time = tic;

    fprintf('Tempo | Posição | Ref. | Ângulo | Controle | Status\n');
    fprintf('------|---------|------|--------|----------|--------\n');

    for i = 1:steps
        tempo_atual = (i-1) * dt;
        
        %% 🎯 CONTROLE DE FASES CORRIGIDO (5cm = 0.05m)
        if i == 1
            fase_atual = 1;
            tempo_inicio_fase = tempo_atual;
        end
        
        % Transição entre fases baseada no tempo
        if fase_atual == 1 && tempo_atual >= 3
            fase_atual = 2;
            tempo_inicio_fase = tempo_atual;
            fprintf('▶️  INICIANDO MOVIMENTO PARA FRENTE\n');
        elseif fase_atual == 2 && tempo_atual >= 8
            fase_atual = 3;
            tempo_inicio_fase = tempo_atual;
            fprintf('▶️  INICIANDO MOVIMENTO PARA TRÁS\n');
        elseif fase_atual == 3 && tempo_atual >= 13
            fase_atual = 4;
            tempo_inicio_fase = tempo_atual;
            fprintf('▶️  RETORNANDO PARA POSIÇÃO ZERO\n');
        end
        
        % Definir referência conforme a fase
        switch fase_atual
            case 1  % Estabilização inicial (3s)
                posicao_desejada = 0;
            case 2  % Mover 5cm para frente (5s)
                posicao_desejada = 0.05;  % 5cm
            case 3  % Mover 5cm para trás (5s)  
                posicao_desejada = -0.05; % -5cm
            case 4  % Retornar para zero
                posicao_desejada = 0;
        end

        %% Medição com tratamento de erro robusto
        try
            pos_data = sim.getObjectPosition(carrinho_handle, -1);
            orient_data = sim.getObjectOrientation(carrinho_handle, -1);
            
            if iscell(pos_data)
                pos_x = double(pos_data{1});
            else
                pos_x = double(pos_data(1));
            end
            
            if iscell(orient_data)
                current_angle = double(orient_data{2});
            else
                current_angle = double(orient_data(2));
            end
            
        catch
            pos_x = last_pos;
            current_angle = last_angle;
        end

        %% Cálculo de derivadas
        current_time = toc(last_time);
        dt_actual = max(current_time, 0.001);
        
        vel_x = (pos_x - last_pos) / dt_actual;
        raw_ang_vel = (current_angle - last_angle) / dt_actual;
        
        % Filtro suave
        if i == 1
            angle_filter = current_angle;
            ang_vel_filter = raw_ang_vel;
        else
            angle_filter = alpha * angle_filter + (1-alpha) * current_angle;
            ang_vel_filter = alpha * ang_vel_filter + (1-alpha) * raw_ang_vel;
        end

        %% 🎯 CONTROLE LQR CORRIGIDO
        erro_posicao = pos_x - posicao_desejada;
        x = [erro_posicao; vel_x; angle_filter; ang_vel_filter];
        u = K * x;
        
        % Limites de controle
        u_limited = max(min(u, 15), -15);
        
        % Conversão para velocidade com ganho ajustado
        target_velocity = u_limited * velocity_gain;
        
        %% Aplicação do controle
        try
            sim.setJointTargetVelocity(jointR_handle, target_velocity);
            sim.setJointTargetVelocity(jointL_handle, target_velocity);
        catch
            fprintf('⚠️  Erro no envio de controle\n');
        end

        %% Atualização e logging
        last_pos = pos_x;
        last_angle = current_angle;
        last_time = tic;
        
        time_log(i) = tempo_atual;
        position_log(i) = pos_x;
        angle_log(i) = angle_filter;
        control_log(i) = u_limited;
        velocity_log(i) = vel_x;
        ang_velocity_log(i) = ang_vel_filter;
        reference_log(i) = posicao_desejada;

        %% Display informativo
        angulo_deg = rad2deg(angle_filter);
        
        % Status das fases
        switch fase_atual
            case 1
                fase_str = 'ESTABILIZAÇÃO';
            case 2
                fase_str = 'FRENTE 5cm';
            case 3
                fase_str = 'TRÁS 5cm';
            case 4
                fase_str = 'ZERO';
        end
        
        if abs(angulo_deg) < 3
            status = '✅ EQUILIBRADO';
        elseif abs(angulo_deg) < 10
            status = '⚠️  OSCILANDO';
        elseif abs(angulo_deg) < 25
            status = '🔶 INSTÁVEL';
        else
            status = '❌ QUASE CAINDO';
        end
        
        if mod(i, 15) == 0 || abs(angulo_deg) > 10
            fprintf('%5.1f | %7.3f | %5.2f | %6.1f° | %8.3f | %s (%s)\n', ...
                    tempo_atual, pos_x, posicao_desejada, angulo_deg, u_limited, status, fase_str);
        end

        % Critério de parada por segurança
        if abs(angulo_deg) > 70
            fprintf('\n💥 Queda detectada - Ângulo: %.1f°\n', angulo_deg);
            break;
        end
        
        client.step();
        pause(dt * 0.1);
    end
    
catch ME
    fprintf('Erro durante simulação: %s\n', ME.message);
end

%% Finalização
try
    sim.stopSimulation();
    pause(1);
catch
end

%% 📊 Análise de resultados
if exist('i', 'var') && i > 10
    effective_steps = min(i, steps);
    
    fprintf('\n📊 RESULTADOS DO MOVIMENTO:\n');
    fprintf('Tempo total: %.1f s\n', time_log(effective_steps));
    fprintf('Posição final: %.3f m\n', position_log(effective_steps));
    fprintf('Referência final: %.3f m\n', reference_log(effective_steps));
    
    % Cálculo de desempenho
    angulo_deg_log = rad2deg(angle_log(1:effective_steps));
    tempo_estavel = sum(abs(angulo_deg_log) < 5) * dt;
    percentual = (tempo_estavel / time_log(effective_steps)) * 100;
    
    fprintf('Tempo estável (<5°): %.1f s (%.1f%%)\n', tempo_estavel, percentual);
    fprintf('Ângulo máximo: %.1f°\n', max(abs(angulo_deg_log)));
    fprintf('Erro final de posição: %.3f m\n', abs(position_log(effective_steps) - reference_log(effective_steps)));
    
    %% Gráficos
    figure('Position', [100, 100, 1200, 800]);
    
    subplot(2,2,1);
    plot(time_log(1:effective_steps), position_log(1:effective_steps), 'b-', 'LineWidth', 2);
    hold on;
    plot(time_log(1:effective_steps), reference_log(1:effective_steps), 'r--', 'LineWidth', 2);
    ylabel('Posição (m)'); grid on;
    title('POSIÇÃO DO CARRINHO');
    legend('Real', 'Referência', 'Location', 'best');
    ylim([-0.07, 0.07]);
    
    subplot(2,2,2);
    plot(time_log(1:effective_steps), angulo_deg_log, 'r-', 'LineWidth', 2);
    ylabel('Ângulo (°)'); grid on;
    title('ÂNGULO DO PÊNDULO');
    yline(5, 'g--', 'Limite estável'); yline(-5, 'g--');
    
    subplot(2,2,3);
    plot(time_log(1:effective_steps), control_log(1:effective_steps), 'g-', 'LineWidth', 2);
    ylabel('Controle (V)'); xlabel('Tempo (s)'); grid on;
    title('TENSÃO DE CONTROLE');
    
    subplot(2,2,4);
    plot(time_log(1:effective_steps), rad2deg(ang_velocity_log(1:effective_steps)), 'm-', 'LineWidth', 2);
    ylabel('Vel. Angular (°/s)'); xlabel('Tempo (s)'); grid on;
    title('VELOCIDADE ANGULAR');
    
    sgtitle('CONTROLE LQR - MOVIMENTO DE 5cm', 'FontSize', 14);
end

fprintf('\n🎯 SIMULAÇÃO CONCLUÍDA!\n');



VERSÃO COM GRÁFICOS

% controle_carrinho_pendulo_tuned.m
% Mudanças mínimas para aumentar o tempo em pé do pêndulo:
% - ganhos PD ajustados (menos P, mais D)
% - filtro de ângulo mais responsivo e filtro separado para velocidade angular
% - ganho de conversão para rodas um pouco maior
% - suavização do comando para evitar "pulsos" bruscos nas rodas
% Salve e execute.
clear all; close all; clc;
%% Inicializacao CoppeliaSim
try
   client = RemoteAPIClient();
   sim = client.require('sim');
   client.setStepping(true);
   jointR_handle = sim.getObject('/Revolute_joint_right');
   jointL_handle = sim.getObject('/Revolute_joint_left');
   carrinho_handle = sim.getObject('/CARRINHO_estrutura');
   fprintf('Conectado ao CoppeliaSim\n');
catch ME
   error(['Erro na conexao: ' ME.message]);
end
%% Parametros do modelo
M = 0.5;      % massa do carrinho (kg)
m = 0.2;      % massa do pendulo (kg)
l = 0.3;      % comprimento (m)
g = 9.81;     % gravidade
bx = 2.5;     % atrito viscoso
bo = 1.0;     % amortecimento angular
km = 0.16;    % constante do motor
%% Espaco de estados (linearizado)
A22 = -bx/M;
A23 = -(m*g)/M;
A24 = -(bo*M*l);
A42 = bx/M;
A43 = (g*(M+m))/(M*l);
A44 = -(bo*(M+m))/(M*m*l*l);
B2 = km/M;
B4 = -km/(M*l);
A = [0  1   0   0;
    0 A22 A23 A24;
    0  0   0   1;
    0 A42 A43 A44];
B = [0; B2; 0; B4];
%% LQR (referencia)
q1 = 100; q2 = 10; q3 = 2000; q4 = 50; r = 0.1;
Q = diag([q1, q2, q3, q4]);
R = r;
K = lqr(A, B, Q, R);
fprintf('Ganhos LQR (recalculado): K = [%.3f, %.3f, %.3f, %.3f]\n', K(1), K(2), K(3), K(4));
%% Configuracao da simulacao
dt = 0.05;
simulation_time = 20;
steps = floor(simulation_time / dt);
% logs
time_log = zeros(steps, 1);
position_log = zeros(steps, 1);
angle_log = zeros(steps, 1);
control_log = zeros(steps, 1);
velocity_log = zeros(steps, 1);
ang_velocity_log = zeros(steps, 1);
reference_log = zeros(steps, 1);
%% Parametros do controlador e sinais (ajustes pontuais)
velocity_gain = 0.12;  % aumentei (de 0.08) para permitir correções um pouco mais fortes
alpha = 0.08;          % filtro do angulo: mais responsivo (antes 0.3)
alpha_vel = 0.28;      % filtro para velocidade angular (separa do filtro do ângulo)
wheel_sign_right = 1;
wheel_sign_left = 1;
%% Limites e ganhos PD ajustados (mínimas mudanças)
k_theta = 55;    % proporcional no angulo (reduzido de 80)
k_omega = 6.0;   % derivativo aumentado (de 4.0) para maior amortecimento
k_pos = 6.0;
k_vel = 0.2;
% escala da contribuicao da posicao (menos agressivo que antes)
angle_scale_deg_threshold = 40;  % antes 20 -> agora 40 (posicao continua atuando mais tempo)
u_max = 25;
max_allowed_angle_deg = 70;
% pequeno filtro/suavização do comando para as rodas
prev_target_velocity = 0;
cmd_smooth_alpha = 0.35; % 0 = uso total de prev, 1 = uso total do novo -> valor intermediario suaviza
%% Inicia simulacao
fprintf('=== INICIANDO CONTROLE (tuned) ===\n');
try
   sim.startSimulation();
   pause(1);
   % posicao inicial
   pos_data = sim.getObjectPosition(carrinho_handle, -1);
   if iscell(pos_data)
       pos_inicial = double(pos_data{1});
   else
       pos_inicial = double(pos_data(1));
   end
   last_pos = pos_inicial;
   last_angle = 0;
   last_time = tic;
   for i = 1:steps
       tempo_atual = (i-1) * dt;
       % referencia por fases (simples)
       if tempo_atual < 3
           pos_ref = 0;
       elseif tempo_atual < 8
           pos_ref = 0.05;
       elseif tempo_atual < 13
           pos_ref = -0.05;
       else
           pos_ref = 0;
       end
       % medicoes
       try
           pos_data = sim.getObjectPosition(carrinho_handle, -1);
           orient_data = sim.getObjectOrientation(carrinho_handle, -1);
           if iscell(pos_data)
               pos_x = double(pos_data{1});
           else
               pos_x = double(pos_data(1));
           end
           if iscell(orient_data)
               current_angle = double(orient_data{2});
           else
               current_angle = double(orient_data(2));
           end
       catch
           pos_x = last_pos;
           current_angle = last_angle;
       end
       % derivadas
       dt_actual = max(toc(last_time), 1e-3);
       vel_x = (pos_x - last_pos) / dt_actual;
       raw_ang_vel = (current_angle - last_angle) / dt_actual;
       if i == 1
           angle_filter = current_angle;
           ang_vel_filter = raw_ang_vel;
       else
           % filtros separados: ângulo mais responsivo, velocidade angular um pouco mais suave
           angle_filter = alpha * angle_filter + (1-alpha) * current_angle;
           ang_vel_filter = alpha_vel * ang_vel_filter + (1-alpha_vel) * raw_ang_vel;
       end
       % Estado e estrategia de controle (prioriza estabilidade angular)
       erro_pos = pos_x - pos_ref;
       x = [erro_pos; vel_x; angle_filter; ang_vel_filter];
       % Controlador hibrido: PD no angulo + termo de posicao
       control_ang = -k_theta * angle_filter - k_omega * ang_vel_filter;
       control_pos = k_pos * (pos_ref - pos_x) - k_vel * vel_x;
       % escala a contribuicao da posicao conforme o angulo (menos agressivo)
       angle_deg = abs(rad2deg(angle_filter));
       angle_scale = max(0, 1 - angle_deg/angle_scale_deg_threshold);
       combined = control_ang + angle_scale * control_pos;
       % opcional: combinar com LQR (mantive apenas o PD para alteração mínima)
       % u_lqr = -K * x;
       % blend_w = 0.0; % 0 = só PD, 1 = só LQR (deixe 0 para mudanças mínimas)
       % u = blend_w * u_lqr + (1-blend_w) * combined;
       u = combined;
       u = max(min(u, u_max), -u_max);
       target_velocity = u * velocity_gain;
       % suaviza comando para evitar saltos bruscos
       target_velocity = cmd_smooth_alpha * target_velocity + (1-cmd_smooth_alpha) * prev_target_velocity;
       prev_target_velocity = target_velocity;
       % aplica comando as rodas
       sim.setJointTargetVelocity(jointR_handle, wheel_sign_right * target_velocity);
       sim.setJointTargetVelocity(jointL_handle, wheel_sign_left  * target_velocity);
       % logs e atualizacoes
       last_pos = pos_x;
       last_angle = current_angle;
       last_time = tic;
       time_log(i) = tempo_atual;
       position_log(i) = pos_x;
       angle_log(i) = angle_filter;
       control_log(i) = u;
       velocity_log(i) = vel_x;
       ang_velocity_log(i) = ang_vel_filter;
       reference_log(i) = pos_ref;
       if mod(i,10) == 0
           fprintf('[DEBUG] t=%.2f pos=%.3f ref=%.3f ang=%.2f° u=%.3f targ_vel=%.3f\n', ...
               tempo_atual, pos_x, pos_ref, rad2deg(angle_filter), u, target_velocity);
       end
       % seguranca
       if abs(rad2deg(angle_filter)) > max_allowed_angle_deg
           fprintf('Queda detectada (ang=%.1f°). Interrompendo.\n', rad2deg(angle_filter));
           break;
       end
       client.step();
       pause(dt * 0.1);
   end
catch ME
   fprintf('Erro durante a simulacao: %s\n', ME.message);
end
% finaliza
try
   sim.stopSimulation();
   pause(0.5);
catch
end
fprintf('Simulacao finalizada.\n');
% plot simples (opcional)
try
   figure;
   subplot(3,1,1);
   plot(time_log, position_log); hold on; plot(time_log, reference_log, '--'); ylabel('x (m)'); legend('pos','ref');
   subplot(3,1,2);
   plot(time_log, rad2deg(angle_log)); ylabel('angle (deg)');
   subplot(3,1,3);
   plot(time_log, control_log); ylabel('u'); xlabel('t (s)');
catch
end

