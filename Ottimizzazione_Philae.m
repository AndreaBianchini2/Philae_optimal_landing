% Script principale: ottimizzazione del lander Philae

clc; clear;
x1_f = 0.46;            % quota finale desiderata [m]
x2_f = 0;              % velocità finale desiderata [m/s]

% Stima iniziale per p1, p2, p3, tf
P_0 = [-0.00056931, -11, 0.05, 19465.9349];

% Ottimizzazione
options = optimset('Display','iter');
P_opt = fminsearch(@lander_Philae, P_0, options);

% Parametri
m_0 = 98;
T_max = 1;
g_0 = 9.81;
g_l = 0.0001;
Isp = 60;
c = g_0 * Isp;

% Condizioni iniziali
x1_0 = 22500;
x2_0 = -0.187;
x3_0 = m_0;
X_0 = [x1_0 x2_0 x3_0];
I = [X_0 P_0];

% Simulazione ottimale
[t, X] = ode113(@(t,X) lander(t,X,T_max,g_l,c), [0 I(7)], I(1:6));

% Estrazione variabili
x1 = X(:,1);
x2 = X(:,2);
x3 = X(:,3);
p2 = X(:,5);
p3 = X(:,6);
theta = p2 ./ x3 - p3 / c;
u = double(theta > 0);
T = T_max * u;
% Hamiltoniana %  
  H = X(:,4) .* X(:,2) + X(:,5) .* (T_max * u ./ X(:,3) - g_l) - X(:,6) .* (T_max * u / c);
% Plot dei risultati

figure(1)
subplot(2,2,1);
plot(t, x1);
xlabel('Tempo [s]'); ylabel('Quota [m]');
title('Quota');

subplot(2,2,2);
plot(t, x2);
xlabel('Tempo [s]'); ylabel('Velocità [m/s]');
title('Velocità');

subplot(2,2,3);
plot(t, x3);
xlabel('Tempo [s]'); ylabel('Massa [kg]');
title('Massa');

subplot(2,2,4);
plot(t, T);
xlabel('Tempo [s]'); ylabel('Spinta [N]');
title('Spinta');

figure(2)
plot(t,theta);
xlabel('tempo'); ylabel('Theta')
title('theta')

err_x1 = x1(end) - x1_f;
err_x2 = x2(end) - x2_f;
disp([err_x1 err_x2]);

figure(3)
plot(t,H);
xlabel('tempo'); ylabel('H')
title('Hamiltoniana')

figure;
plot(t,x3, 'b', 'LineWidth', 2);
xlabel('Tempo [s]');
ylabel('Massa [kg]');
title('Zoom sulla variazione della massa');
xlim([19330 19470]); % <-- intervallo di tempo della variazione
ylim([97.75 98]);    % <-- restringi anche l’asse Y per accentuare la pendenza
grid on;