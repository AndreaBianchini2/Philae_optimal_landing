%equazioni dello stato e del costato integrate da ode113

function f = lander(~, X, T_max, g_l, c)
    f = zeros(6,1);
    theta = X(5)/X(3) - X(6)/c;
    u = double(theta > 0);
    f(1) = X(2);
    f(2) = T_max * u / X(3) - g_l;
    f(3) = -T_max * u / c;
    f(4) = 0;
    f(5) = -X(4);
    f(6) = (X(5) * T_max * u) / (X(3)^2);
end