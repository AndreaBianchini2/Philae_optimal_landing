% funzione costo da minimizzare

function J = lander_Philae(P_0)
    % P_0 = [p1_0, p2_0, p3_0, tf]

    % Parametri fisici
    m_0 = 98;           % massa iniziale [kg]
    T_max = 1;         % spinta massima [N]
    g_0 = 9.81;            % gravità terrestre [m/s^2]
    g_l =  0.0001;            % gravità cometa [m/s^2]
    Isp = 60;             % impulso specifico [s]
    c = g_0 * Isp;

    % Condizioni iniziali
    x1_0 = 22500;           % quota iniziale [m]
    x2_0 = -0.187;            % velocità iniziale [m/s]
    x3_0 = m_0;            % massa iniziale
    X_0 = [x1_0 x2_0 x3_0];
    I = [X_0 P_0];         % stato iniziale completo: x1, x2, x3, p1, p2, p3, tf

    % Obiettivi finali
    x1_f = 0.46;
    x2_f = 0;
   
               

    % Simulazione dinamica
    [~, X] = ode113(@(t,X) lander(t,X,T_max,g_l,c), [0 I(7)], I(1:6));

    % Estrazione variabili
    x3 = X(:,3);
    p2 = X(:,5);
    p3 = X(:,6);

    % Controllo
    theta = p2 ./ x3 - p3 / c;
    u = double(theta > 0);
    % Hamiltoniana
    H = X(:,4) .* X(:,2) + X(:,5) .* (T_max * u ./ X(:,3) - g_l) - X(:,6) .* (T_max * u / c);

    % Funzione costo (criterio di ottimalità)
   c1 = 1; c2 = 1; c3 = 0.001; c4 = 0.001;
   J = c1*abs(X(end,1) - x1_f) + ...
        c2*abs(X(end,2) - x2_f) + ...
        c3*abs(X(end,6) - 1) + ...
        c4*abs(H(end));
end