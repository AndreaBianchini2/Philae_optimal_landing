function J = CostFunction(P)

% Parametri lander
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

% Obiettivi finali
x1_f = 0.46;
x2_f = 0;

try
    % Simulazione
    [t, X] = ode113(@(t,X) lander(t,X,T_max,g_l,c), [0 P(4)], [x1_0 x2_0 x3_0 P(1:3)]);

    x1 = X(:,1);
    x2 = X(:,2);
    x3 = X(:,3);
    p2 = X(:,5);
    p3 = X(:,6);
    theta = p2 ./ x3 - p3 / c;

    % Controllo ON/OFF
    u = double(theta > 0);
    T = T_max * u;

    % Errore finale
    err1 = x1(end) - x1_f;
    err2 = x2(end) - x2_f;

    % Penalità: accensione troppo presto
    idx_on = find(u > 0, 1, 'first');
    if isempty(idx_on)
        penalty = 1000;
    else
        t_switch = t(idx_on);
        tf = t(end);
        penalty = 10 * (1 - t_switch / tf);  % + grande se spinta parte presto
    end

    % Costo totale
    J = err1^2 + err2^2 + penalty;

catch
    J = 1e6; % penalizza errori di integrazione
end
end