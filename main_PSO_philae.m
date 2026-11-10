clc; clear;

% === Parametri PSO ===
nPop = 1000;            % numero particelle
MaxIter = 100;         % massimo iterazioni
w = 0.7;              % inerzia
c1 = 1; c2 = 3;    % coefficienti cognitivi e sociali

nVar = 4;                             % [p1_0, p2_0, p3_0, tf]
VarSize = [1 nVar];
VarMin = [-0.00058, -9, 0.05, 19465];  % limiti inferiori
VarMax = [-0.00054, -11, 0.05, 19466];     % limiti superiori

% === Inizializzazione particelle ===
for i = 1:nPop
    % uso rand invece di unifrnd
    particle(i).Position = VarMin + (VarMax - VarMin) .* rand(1, nVar);
    particle(i).Velocity = zeros(VarSize);
    particle(i).Cost = CostFunction(particle(i).Position);
    particle(i).Best.Position = particle(i).Position;
    particle(i).Best.Cost = particle(i).Cost;
end

% Best globale iniziale
[~, idx] = min([particle.Cost]);
GlobalBest = particle(idx).Best;

% === Ciclo PSO ===
for it = 1:MaxIter
    for i = 1:nPop
        % Aggiorna velocità
        particle(i).Velocity = ...
            w*particle(i).Velocity ...
            + c1*rand(VarSize).*(particle(i).Best.Position - particle(i).Position) ...
            + c2*rand(VarSize).*(GlobalBest.Position - particle(i).Position);

        % Aggiorna posizione
        particle(i).Position = particle(i).Position + particle(i).Velocity;

        % Limita valori
        particle(i).Position = max(particle(i).Position, VarMin);
        particle(i).Position = min(particle(i).Position, VarMax);

        % Valuta costo
        particle(i).Cost = CostFunction(particle(i).Position);

        % Aggiorna best personale
        if particle(i).Cost < particle(i).Best.Cost
            particle(i).Best.Position = particle(i).Position;
            particle(i).Best.Cost = particle(i).Cost;

            % Aggiorna best globale
            if particle(i).Best.Cost < GlobalBest.Cost
                GlobalBest = particle(i).Best;
            end
        end
    end

    % Visualizza progresso
    fprintf('Iterazione %d: Costo = %.6f\n', it, GlobalBest.Cost);
end

% === Risultato finale ===
P_opt = GlobalBest.Position;
disp('Condizioni ottimali trovate:');
disp(['p1(0): ' num2str(P_opt(1))]);
disp(['p2(0): ' num2str(P_opt(2))]);
disp(['p3(0): ' num2str(P_opt(3))]);
disp(['t_f  : ' num2str(P_opt(4))]);

% === Simulazione finale ===
m_0 = 98; g_0 = 9.81; g_l = 0.0001; Isp = 60;
c = g_0 * Isp;
x1_0 = 22500; x2_0 = -0.187;
[t, X] = ode113(@(t,X) lander(t,X,1,g_l,c), [0 P_opt(4)], [x1_0 x2_0 m_0 P_opt(1:3)]);

x1 = X(:,1); x2 = X(:,2); x3 = X(:,3);
p2 = X(:,5); p3 = X(:,6);
theta = p2 ./ x3 - p3 / c;
u = double(theta > 0);

% === Plot ===
figure;
subplot(2,2,1); plot(t,x1); title('Quota'); ylabel('m'); xlabel('s');
subplot(2,2,2); plot(t,x2); title('Velocità'); ylabel('m/s'); xlabel('s');
subplot(2,2,3); plot(t,x3); title('Massa'); ylabel('kg'); xlabel('s');
subplot(2,2,4); plot(t,u); title('Spinta ON/OFF'); xlabel('s');

figure;
plot(t, theta); title('\theta(t)'); xlabel('s');

idx_on = find(u > 0, 1, 'first');
if ~isempty(idx_on)
    hold on
    plot(t(idx_on), x1(idx_on), 'ro', 'MarkerSize', 8, 'DisplayName', 'Spinta attiva')
    legend
end