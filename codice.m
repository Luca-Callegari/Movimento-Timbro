clc
close all
clear all
%% NON SUPPONIAMO ORA CHE DEVO AVERE VELOCITA' ORTOGONALI NELLA CONFIGURAZIONE FINALE

%% Definizione dei punti iniziali e finali + calcolo mediane

% Punti configurazione finale/iniziale
A1 = [193;  57];
A2 = [120;  10];
B1 = [203;  77];
B2 = [140;  0];

% Calcolo dei punti medi
M_A = (A1 + A2) / 2;
M_B = (B1 + B2) / 2;

% Pendenze dei segmenti
m_A = (A2(2)-A1(2)) / (A2(1)-A1(1));
m_B = (B2(2)-B1(2)) / (B2(1)-B1(1));

% Pendenze delle mediane ortogonali
m_perp_A = -1/m_A;
m_perp_B = -1/m_B;

% Intercette delle mediane ortogonali
q_perp_A = M_A(2) - m_perp_A*M_A(1);
q_perp_B = M_B(2) - m_perp_B*M_B(1);

% Calcolo equazioni delle mediane A1A2 e B1B2 per trovare informazioni
% delle posizioni di A0 e B0 (cerniere fisse)

yA0 = @(xA0) m_perp_A*xA0 + q_perp_A;
yB0 = @(xB0) m_perp_B*xB0 + q_perp_B;

hold on
grid on
h1 = fplot(yA0,[0 203], "m-");
h2 = fplot(yB0,[0 203], "k-");
legend([h1, h2], {'Mediana A1A2', 'Mediana B1B2'}, 'Location', 'best');
xlabel('x');
ylabel('y');
title('Assi dei segmenti A1A2 e B1B2');

% Scelta arbitraria delle ordinate di A0 e B0
xA0 = 100;
xB0 = 80;

A0 = [xA0; yA0(xA0)];
B0 = [xB0; yB0(xB0)];

%% Uso della matrice di spostamento per vedere traiettoria dei punti A(t) e B(t)

% Funzione per calcolo angolo diretto con orientamento corretto
angle_between = @(v1, v2) atan2( ...
    v1(1)*v2(2) - v1(2)*v2(1), ...
    v1(1)*v2(1) + v1(2)*v2(2));

theta_A = angle_between(A1 - A0, A2 - A0);
theta_B = angle_between(B1 - B0, B2 - B0);

fprintf('theta_A = %.6f rad (%.3f deg)\n', theta_A, theta_A*180/pi);
fprintf('theta_B = %.6f rad (%.3f deg)\n', theta_B, theta_B*180/pi);

% Definisco MATRICE DI SPOSTAMENTO D(θ,p)

D = @(theta, p) [ ...
    cos(theta)  -sin(theta)   p(1)*(1-cos(theta)) + p(2)*sin(theta);
    sin(theta)   cos(theta)   p(2)*(1-cos(theta)) - p(1)*sin(theta);
    0            0            1 ];

% Numero passi discreti di calcolo punti traiettorie
num_steps = 100;
t = linspace(0,1,num_steps);

A_traj = zeros(2, num_steps);
B_traj = zeros(2, num_steps);

% Calcolo punti delle traiettorie A(t) e B(t)
for k = 1:num_steps
    DtA = D(theta_A * t(k), A0);
    DtB = D(theta_B * t(k), B0);

    % converto in coordinate omogenee
    A_h = [A1; 1];
    B_h = [B1; 1];

    % applico la matrice di spostamento
    A_new = DtA * A_h;
    B_new = DtB * B_h;

    A_traj(:,k) = A_new(1:2);
    B_traj(:,k) = B_new(1:2);
end

figure; hold on; grid on; axis equal;

plot(A_traj(1,:), A_traj(2,:), 'b', 'LineWidth', 2);
plot(B_traj(1,:), B_traj(2,:), 'r', 'LineWidth', 2);

plot(A1(1),A1(2),'bo', 'MarkerFaceColor','b');
plot(A2(1),A2(2),'bs', 'MarkerFaceColor','b');
plot(B1(1),B1(2),'ro', 'MarkerFaceColor','r');
plot(B2(1),B2(2),'rs', 'MarkerFaceColor','r');

plot(A0(1),A0(2),'ko','MarkerFaceColor','k');
plot(B0(1),B0(2),'ko','MarkerFaceColor','k');

legend('Traiettoria A','Traiettoria B',...
       'A1','A2','B1','B2','A0/B0');

title('Traiettorie generate tramite Matrice di Spostamento');


%%  SIMULAZIONE QUADRILATERO (A0-A movente)

% ---- Lunghezze costanti ----
L1 = norm(A1 - A0);   % A0-A
L2 = norm(A1 - B1);   % A-B   (BIELLA)
L3 = norm(B1 - B0);   % B0-B
L4 = norm(A0 - B0);   % A0-B0

% ---- Calcolo angoli di rotazione ----
angle_between = @(v1, v2) atan2( ...
    v1(1)*v2(2) - v1(2)*v2(1), ...
    v1(1)*v2(1) + v1(2)*v2(2));

theta_A = angle_between(A1 - A0, A2 - A0);
theta_B = angle_between(B1 - B0, B2 - B0);

% ---- Matrice di spostamento ----
D = @(theta,p) [ ...
    cos(theta) -sin(theta) p(1)*(1-cos(theta)) + p(2)*sin(theta);
    sin(theta)  cos(theta) p(2)*(1-cos(theta)) - p(1)*sin(theta);
    0 0 1 ];

% ---- Preparazione traiettorie ----
num_steps = 120;
t = linspace(0,1,num_steps);

A_traj = zeros(2,num_steps);
B_traj = zeros(2,num_steps);

for k = 1:num_steps
    % --- Calcolo A(t) con matrice di trasformazione ---
    DtA = D(theta_A*t(k), A0);
    A = DtA * [A1; 1];
    A = A(1:2);
    A_traj(:,k) = A;

    % --- Calcolo B(t) come intersezione cerchi ---
    % Cerchio 1: centro A, raggio L2
    % Cerchio 2: centro B0, raggio L3
    d = norm(B0 - A);

    % formula standard intersezione cerchi
    a = (L2^2 - L3^2 + d^2) / (2*d);
    h = sqrt(L2^2 - a^2);

    P2 = A + a*(B0 - A)/d;

    % Soluzione (scegliamo quella coerente con B1)
    temp = B0 - A;
    perp = [-temp(2); temp(1)] / d;

    Bsol1 = P2 + h*perp;
    Bsol2 = P2 - h*perp;

    % scegli la soluzione più vicina a B1 nella prima parte
    if k == 1
        if norm(Bsol1 - B1) < norm(Bsol2 - B1)
            choose = 1;
        else
            choose = 2;
        end
    end

    B = Bsol1;
    if choose == 2
        B = Bsol2;
    end

    B_traj(:,k) = B;
end

% ======== Animazione quadrilatero ========

figure; hold on; axis equal; grid on;
title("Quadrilatero articolato con lunghezze L1,L2,L3 costanti");

for k = 1:num_steps
    A = A_traj(:,k);
    B = B_traj(:,k);

    cla;

    plot(A0(1),A0(2),"ko","MarkerFaceColor","k");
    plot(B0(1),B0(2),"ko","MarkerFaceColor","k");

    plot([A0(1) A(1)], [A0(2) A(2)], 'r', 'LineWidth', 2);
    plot([A(1) B(1)], [A(2) B(2)], 'g', 'LineWidth', 2);
    plot([B0(1) B(1)], [B0(2) B(2)], 'b', 'LineWidth', 2);

    plot(A(1),A(2),"ro","MarkerFaceColor","r");
    plot(B(1),B(2),"bo","MarkerFaceColor","b");

    xlim([0 250]);
    ylim([0 200]);

    pause(0.02);
end

% Stampa configurazioni iniziali per A
fprintf("Configurazione iniziale nominale A1 = [%d %d]\n", A1(1), A1(2));
fprintf("Configurazione iniziale reale    A1 = [%d %d]\n", A_traj(1,1), A_traj(2,1));

% Stampa configurazioni iniziali per B
fprintf("Configurazione iniziale nominale B1 = [%d %d]\n", B1(1), B1(2));
fprintf("Configurazione iniziale reale    B1 = [%d %d]\n", B_traj(1,1), B_traj(2,1));

% Stampa configurazioni finali per verifica
fprintf("Configurazione finale reale A2 = [%d %d]\n", A_traj(1,end), A_traj(2,end));
fprintf("Configurazione finale reale B2 = [%d %d]\n", B_traj(1,end), B_traj(2,end));


%% SUPPONIAMO ORA CHE DEVO AVERE VELOCITA' ORTOGONALI NELLA CONFIGURAZIONE FINALE
clc
close all
clear all

% Punti configurazione finale/iniziale
% Ho scelto questa configurazione iniziale delle cerniere mobili cosicché
% la configurazione finale delle cerniere mobili presenti A2 a quota zero,
% il perché lo dico dopo.
deltaA = 0;
deltaB = 15;
A1 = [203;  57 + deltaA];
A2 = [120 + deltaA;  0];
B1 = [193;  77 + deltaB];
B2 = [140 + deltaB;  10];

% Calcolo dei punti medi
M_A = (A1 + A2) / 2;
M_B = (B1 + B2) / 2;

% Pendenze dei segmenti
m_A = (A2(2)-A1(2)) / (A2(1)-A1(1));
m_B = (B2(2)-B1(2)) / (B2(1)-B1(1));

% Pendenze delle mediane ortogonali
m_perp_A = -1/m_A;
m_perp_B = -1/m_B;

% Intercette delle mediane ortogonali
q_perp_A = M_A(2) - m_perp_A*M_A(1);
q_perp_B = M_B(2) - m_perp_B*M_B(1);

% Calcolo equazioni delle mediane A1A2 e B1B2 per trovare informazioni
% delle posizioni di A0 e B0 (cerniere fisse)
yA0 = @(xA0) m_perp_A*xA0 + q_perp_A;
yB0 = @(xB0) m_perp_B*xB0 + q_perp_B;

hold on
grid on
h1 = fplot(yA0,[0 203], "m-");
h2 = fplot(yB0,[0 203], "k-");
legend([h1, h2], {'Mediana A1A2', 'Mediana B1B2'}, 'Location', 'best');
xlabel('x');
ylabel('y');
title('Assi dei segmenti A1A2 e B1B2');
% Da questo plot vediamo che la cerniera fissa A0 la possiamo mettere anche
% lei, come A2, a quota zero, così da avere P31 nel piano di stampa.
% Per definizione P31 deve essere l'intersezione dei prolungamenti di A0A2
% e B0B2. Quindi se fisso A0 a quota nulla, una qualsiasi B0 nella mediana
% di B1B2 permette velocità ortogonali di tutti i punti del timbro sul
% piano di stampa.

%% Scelta posizione cerniere fisse

% Scelgo dal commento di prima yA0 = 0 e xB0 in modo tale da non avere P31
% dove è presente il timbro nella configurazione 2. Non lo vogliamo perché
% altrimenti, se per assurdo P31 si trova a quota zero ma "dentro" il
% timbro, allora esisteranno sia punti con velocità ortogonale verso l'alto
% sia verso il basso...Noi vogliamo che tutte le velocità sia siano
% equiverse.
yA0 = 0;
xA0 =@(yA0) (yA0-q_perp_A)/m_perp_A;
xB0 = 185;
yB0 = @(xB0) m_perp_B*xB0 + q_perp_B;

A0 = [xA0(yA0); yA0];
B0 = [xB0; yB0(xB0)];

%% Range angolo membro movente

% Angolo theta2 nella configurazione finale
theta2_end = atan2(A2(2)-A0(2), A2(1)-A0(1)) - atan2(B0(2)-A0(2), B0(1)-A0(1));

% Angolo theta2 nella configurazione iniziale
theta2_start = atan2(A1(2)-A0(2), A1(1)-A0(1)) - atan2(B0(2)-A0(2), B0(1)-A0(1));

% Range angolo theta2

%theta2_range = [theta2_start theta2_end];
num_steps = 120;
theta2_range = linspace(theta2_start, theta2_end, num_steps);


%% Funzione che dato angolo theta2 calcola theta3 e theta4

function F = angoli(theta2,L1,L2,L3,L4)

    % A, B, C
    A = 2*L4*(L2*cos(theta2) - L1);
    B = 2*L2*L4*sin(theta2);
    C = L1^2 + L2^2 - L3^2 + L4^2 - 2*L1*L2*cos(theta2);

    % discriminante
    D = A^2 + B^2 - C^2;
    if D < 0
        error('Configurazione impossibile: discriminante negativo.');
    end

    theta4 = 2 * atan2(B - sqrt(D), A - C);

    if theta4 < 0 
        theta4 = theta4 + 2*pi;
    end

    % theta3 sempre in radianti
    theta3 = acos((L1-L2*cos(theta2) - L4*cos(theta4)) / L3);

    F = [theta3 theta4];
end

%% SIMULAZIONE QUADRILATERO
% ---- Lunghezze costanti ----
L1 = norm(A1 - A0);   % A0-A
L2 = norm(A1 - B1);   % A-B   (BIELLA)
L3 = norm(B1 - B0);   % B0-B
L4 = norm(A0 - B0);   % A0-B0

L = [L1 L2 L3 L4];
MAX = max(L);
MIN = min(L);
L(L==max(L))=[];
L(L==min(L))=[];

if MAX+MIN > L(1)+L(2)
    disp("E' un doppio bilancere!");
end

% Calcolo angoli di rotazione 
angle_between = @(v1, v2) atan2( ...
    v1(1)*v2(2) - v1(2)*v2(1), ...
    v1(1)*v2(1) + v1(2)*v2(2));

theta_A = angle_between(A1 - A0, A2 - A0);

% Matrice di spostamento
D = @(theta,p) [ ...
    cos(theta) -sin(theta) p(1)*(1-cos(theta)) + p(2)*sin(theta);
    sin(theta)  cos(theta) p(2)*(1-cos(theta)) - p(1)*sin(theta);
    0 0 1 ];

% Tempo discretizzato
t = linspace(0,1,num_steps);

% Registro traiettorie di A e B
A_traj = zeros(2,num_steps);
B_traj = zeros(2,num_steps);

% Definizione angolo di trasmissione
mu = zeros(num_steps,1);

% Definizione valore determinate dello Jacobiano
val_detJ = zeros(1, num_steps);

%seq = [1:num_steps, num_steps:-1:1];
seq = 1:(num_steps);

% CALCOLO DELLA TRAIETTORIA DI A E B
choose = 0;
for k = seq

    % Calcolo A(t) con matrice di trasformazione
    DtA = D(theta_A*t(k), A0);
    A = DtA * [A1; 1];
    A = A(1:2);
    A_traj(:,k) = A;


    %Calcolo andamento di angolo trasmissione mu
    angoli_theta3_e_theta4_1 = angoli(theta2_range(k),L4,L1,L2,L3);
    if k < 120
        angoli_theta3_e_theta4_2 = angoli(theta2_range(k+1),L4,L1,L2,L3);
    end

    theta3_1 = angoli_theta3_e_theta4_1(1);
    theta3_2 = angoli_theta3_e_theta4_2(1);

    if theta3_2 <= theta3_1
        theta3 = 2*pi - theta3_1;
    else
        theta3 = theta3_1;
    end

    theta4 = angoli_theta3_e_theta4_1(2);

    % Formula calcolo dell'angolo di trasmissione
    mu(k,:) =  abs(theta4 - pi - theta3);

    % Calcolo Jacobiano con A0A membro movente
    J = [-L2*sin(theta3) -L3*sin(theta4); L2*cos(theta3) L3*cos(theta4)];
    val_detJ(1,k) = det(J);
    if det(J) == 0
        fprintf("CONFIGURAZIONE SINGOLARE! in (theta2 = %s, theta3 = %s," + ...
            "theta4 = %s)", num2str(theta2), num2str(theta3), num2str(theta4));
    end

    % Calcolo B(t) come intersezione cerchi
    % Cerchio 1: centro A, raggio L2
    % Cerchio 2: centro B0, raggio L3
    d = norm(B0 - A);

    % formula standard intersezione cerchi
    a = (L2^2 - L3^2 + d^2) / (2*d);
    h = sqrt(L2^2 - a^2);

    P2 = A + a*(B0 - A)/d;

    % Soluzione (scegliamo quella coerente con B1)
    temp = B0 - A;
    perp = [-temp(2); temp(1)] / d;

    Bsol1 = P2 + h*perp;
    Bsol2 = P2 - h*perp;

    % scegli la soluzione più vicina a B1 nella prima parte
    if k == 1
        if norm(Bsol1 - B1) < norm(Bsol2 - B1)
            choose = 1;
        else
            choose = 2;
        end
    end


    B = Bsol1;
    if choose == 2
        B = Bsol2;
    end

    B_traj(:,k) = B;
end

% ANIMAZIONE DEL QUADRILATERO
figure; hold on; axis equal; grid on;
title("Animazione quadrilatero articolato");

for k = seq
    A = A_traj(:,k); %Cerniera mobile A
    B = B_traj(:,k); %Cerniera mobile B

    cla;

    plot(A0(1),A0(2),"ko","MarkerFaceColor","k");
    plot(B0(1),B0(2),"ko","MarkerFaceColor","k");

    plot([A0(1) A(1)], [A0(2) A(2)], 'r', 'LineWidth', 2);
    plot([A(1) B(1)], [A(2) B(2)], 'g', 'LineWidth', 2);
    plot([B0(1) B(1)], [B0(2) B(2)], 'b', 'LineWidth', 2);

    plot(A(1),A(2),"ro","MarkerFaceColor","r");
    plot(B(1),B(2),"bo","MarkerFaceColor","b");

    xlim([0 220]);
    ylim([0 163]);

    title(['Configurazione ' num2str(k) ' - Click per continuare'], 'FontSize', 12);
    
    % Aspetta un click del mouse (sinistro, destro o centrale)
    %waitforbuttonpress;
    pause(0.002);
end


%% SIMULAZIONE QUADRILATERO CON POLARI

% Lunghezze membri
L1 = norm(A1 - A0);   % A0-A
L2 = norm(A1 - B1);   % A-B   (BIELLA)
L3 = norm(B1 - B0);   % B0-B
L4 = norm(A0 - B0);   % A0-B0

L = [L1 L2 L3 L4];
MAX = max(L);
MIN = min(L);
L(L==max(L))=[];
L(L==min(L))=[];

if MAX+MIN > L(1)+L(2)
    disp("E' un doppio bilancere!");
end

% Intersezione tra le rette A0-A1 e B0-B1
function P = intersezioneRetta(A0, A1, B0, B1)
        
        v1 = A1 - A0;
        v2 = B1 - B0;
        
        M = [v1(:), -v2(:)];
        
        if abs(det(M)) < 1e-12
            error('Rette parallele');
        end
        
        ts = M \ (B0(:) - A0(:));
        t = ts(1);
        
        P = A0 + t * v1;
    end


% ---- Calcolo angoli di rotazione ----
angle_between = @(v1, v2) atan2( ...
    v1(1)*v2(2) - v1(2)*v2(1), ...
    v1(1)*v2(1) + v1(2)*v2(2));

theta_A = angle_between(A1 - A0, A2 - A0);
theta_B = angle_between(B1 - B0, B2 - B0);

% ---- Matrice di spostamento ----
D = @(theta,p) [ ...
    cos(theta) -sin(theta) p(1)*(1-cos(theta)) + p(2)*sin(theta);
    sin(theta)  cos(theta) p(2)*(1-cos(theta)) - p(1)*sin(theta);
    0 0 1 ];

% ---- Preparazione traiettorie ----
t = linspace(0,1,num_steps);

A_traj = zeros(2,num_steps);
B_traj = zeros(2,num_steps);

% CALCOLO DELLA TRAIETTORIA DI A E B
%seq = [1:num_steps, num_steps:-1:1];
seq = 1:(num_steps);
for k = seq

    % Calcolo A(t) con matrice di trasformazione
    DtA = D(theta_A*t(k), A0);
    A = DtA * [A1; 1];
    A = A(1:2);
    A_traj(:,k) = A;


    % Calcolo B(t) come intersezione cerchi
    % Cerchio 1: centro A, raggio L2
    % Cerchio 2: centro B0, raggio L3
    d = norm(B0 - A);

    % formula standard intersezione cerchi
    a = (L2^2 - L3^2 + d^2) / (2*d);
    h = sqrt(L2^2 - a^2);

    P2 = A + a*(B0 - A)/d;

    % Soluzione (scegliamo quella coerente con B1)
    temp = B0 - A;
    perp = [-temp(2); temp(1)] / d;

    Bsol1 = P2 + h*perp;
    Bsol2 = P2 - h*perp;

    % scegli la soluzione più vicina a B1 nella prima parte
    if k == 1
        if norm(Bsol1 - B1) < norm(Bsol2 - B1)
            choose = 1;
        else
            choose = 2;
        end
    end 

    B = Bsol1;
    if choose == 2
        B = Bsol2;
    end

    B_traj(:,k) = B;
end

% ANIMAZIONE QUADRILATERO

figure; hold on; axis equal; grid on;
title("Quadrilatero articolato");

polare_fissa = zeros(num_steps,2);
polare_mobile1 = zeros(num_steps,2);

for k = seq
    A = A_traj(:,k); %Cerniera mobile A
    B = B_traj(:,k); %Cerniera mobile B

    % Polare fissa (usando Teo Kennedy ad ogni istante di tempo)
    polare_fissa(k,:) = intersezioneRetta(A0, A, B0, B);

    % CIR scritto in colonna
    PP = polare_fissa(k,:)'; 

    %angolo orientamento sistema mobile (solidale alla biella A-B) rispetto al fisso
    theta = atan2(B(2)-A(2), B(1)-A(1));

    % Metodo polare mobile 1
    R = [ cos(theta)  -sin(theta);
          sin(theta)   cos(theta) ];

    PP_new = R'*(PP-A);
    polare_mobile1(k,:) = PP_new';

    % ===== Trasformazione per visualizzazione nel fisso =====
    % Per plottare la polare mobile nel sistema fisso, trasformiamo tutti
    % i punti accumulati della polare mobile dalle coordinate mobili a quelle fisse
    if k > 1
        % Per ogni punto accumulato della polare mobile, lo trasformiamo
        % dalle coordinate mobili (sistema di riferimento della biella)
        % alle coordinate fisse usando la trasformazione attuale
        polare_mobile_nel_fisso = zeros(k, 2);
        for i = 1:k
            % Trasformiamo il punto dalla rappresentazione mobile a quella fissa
            % usando la posizione attuale del sistema mobile
            punto_mobile = polare_mobile1(i,:)';
            punto_fisso = A + R * punto_mobile;
            polare_mobile_nel_fisso(i,:) = punto_fisso';
        end
    end

    cla;

    plot(polare_fissa(1:k,1), polare_fissa(1:k,2), 'k' , 'LineWidth', 2);
     %  Plot della polare mobile nel sistema fisso 
    if k > 1
        plot(polare_mobile_nel_fisso(:,1), polare_mobile_nel_fisso(:,2), 'm' ,'LineWidth', 2);
    end

    plot(A0(1),A0(2),"ko","MarkerFaceColor","k");
    plot(B0(1),B0(2),"ko","MarkerFaceColor","k");

    plot([A0(1) A(1)], [A0(2) A(2)], 'r', 'LineWidth', 2);
    plot([A(1) B(1)], [A(2) B(2)], 'g', 'LineWidth', 2);
    plot([B0(1) B(1)], [B0(2) B(2)], 'b', 'LineWidth', 2);

    plot(A(1),A(2),"ro","MarkerFaceColor","r");
    plot(B(1),B(2),"bo","MarkerFaceColor","b");

    xlim([0 250]);
    ylim([-50 150]);

    pause(0.002);
end

% Stampa configurazioni iniziali per A
fprintf("Configurazione iniziale nominale A1 = [%d %d]\n", A1(1), A1(2));
fprintf("Configurazione iniziale reale    A1 = [%d %d]\n", A_traj(1,1), A_traj(2,1));

% Stampa configurazioni iniziali per B
fprintf("Configurazione iniziale nominale B1 = [%d %d]\n", B1(1), B1(2));
fprintf("Configurazione iniziale reale    B1 = [%d %d]\n", B_traj(1,1), B_traj(2,1));

% Stampa configurazioni finali per verifica
fprintf("Configurazione finale reale A2 = [%d %d]\n", A_traj(1,end), A_traj(2,end));
fprintf("Configurazione finale reale B2 = [%d %d]\n", B_traj(1,end), B_traj(2,end));

%% Andamento dell'angolo di trasmissione e polare fissa

% Plot delle polari
figure()
hold on;
plot(polare_fissa(:,1), polare_fissa(:,2), 'LineWidth', 2);
%plot(polare_mobile_nel_fisso(1:k,1), polare_mobile_nel_fisso(1:k,2), 'y' ,'LineWidth', 2);
xlabel('X');
ylabel('Y');
grid on;
hold off;
axis equal;  

% Plot della mu in funzione del tempo
figure()
mu_degree = mu/pi*180;
plot(mu_degree, 'LineWidth', 2)
xlabel('t');
ylabel('mu')
grid on;
xlim([0 120])
ylim([min(mu_degree)-40 max(mu_degree)]+20)

% Plot della detJ in funzione del tempo
figure()
plot(val_detJ, 'LineWidth', 2)
xlabel('t');
ylabel('detJ')
grid on;
xlim([0 120])
ylim([min(val_detJ)-50 max(val_detJ)])





%% Calcolo circonferenza dei flessi nella configurazione iniziale e finale

function circonferenzaFlessi(A0,B0,A,B)
    
    % Trovo il punto P31, CIR
    P = intersezioneRetta(A0, A, B0, B);
    
    figure; hold on; axis equal; grid on;
    
    
    % Rette A0-A e B0-B
    plot([A0(1) A(1)], [A0(2) A(2)], 'b-');
    plot([B0(1) B(1)], [B0(2) B(2)], 'r-');
    
    % Plotto tutte le cerniere
    plot(A0(1), A0(2), 'bo'); text(A0(1)+1, A0(2)+1, 'A0', 'FontSize', 10);
    plot(A(1), A(2), 'bo'); text(A(1)+1, A(2)+1, 'A',  'FontSize', 10);
    
    plot(B0(1), B0(2), 'ro'); text(B0(1)+1, B0(2)+1, 'B0', 'FontSize', 10);
    plot(B(1), B(2), 'ro'); text(B(1)+1, B(2)+1, 'B',  'FontSize', 10);
    
    % Plotto P31
    plot(P(1), P(2), 'ko', 'MarkerSize', 5, 'MarkerFaceColor','k');
    text(P(1)+1, P(2)+1, 'P31', 'FontSize', 10, 'Color', 'k');
    
    % Calcolo punti di flesso A' e B'
    % Per trovarli uso la seconda formula di Euler Savary, notando che il
    % centro di curvatura di A è proprio A0, stessa cosa per B.
    
    PA = norm(P - A);
    PB = norm(P - B);
    OMEGA_AA = norm(A0 - A); % OMEGA_A-A
    OMEGA_BB = norm(B0 - B); % OMEGA_B-B
    
    A_fless_A = (PA^2)/OMEGA_AA;
    B_fless_B = (PB^2)/OMEGA_BB;
    
    % Vettori direzione normalizzati (da A verso A0 e da B verso B0)
    dirA = (A0 - A) / norm(A0 - A);
    dirB = (B0 - B) / norm(B0 - B);
    
    % Coordinate dei punti di flesso
    A_fless = A + dirA * A_fless_A;
    B_fless = B + dirB * B_fless_B;
    
    % Disegno circonferenza dei flessi
    plot(A_fless(1), A_fless(2), 'mo', 'MarkerSize', 8, 'MarkerFaceColor','m');
    text(A_fless(1)+1, A_fless(2)+1, 'A''', 'Color','m', 'FontSize', 10);
    
    plot(B_fless(1), B_fless(2), 'co', 'MarkerSize', 8, 'MarkerFaceColor','c');
    text(B_fless(1)+1, B_fless(2)+1, 'B''', 'Color','c', 'FontSize', 10);
    
    pbaspect([1 100 1]);
    ylim([-100 170]);
    xlim([50 200]);
    
    function [xc, yc, R] = cerchioTrePunti(P1, P2, P3)
        x1 = P1(1); y1 = P1(2);
        x2 = P2(1); y2 = P2(2);
        x3 = P3(1); y3 = P3(2);
    
        % Sistema lineare per trovare il centro
        A_ = [2*(x2 - x1), 2*(y2 - y1);
             2*(x3 - x1), 2*(y3 - y1)];
        b = [x2^2 - x1^2 + y2^2 - y1^2;
             x3^2 - x1^2 + y3^2 - y1^2];
    
        C = A_\b;  % centro (xc, yc)
        xc = C(1);
        yc = C(2);
    
        % Raggio
        R = sqrt((xc - x1)^2 + (yc - y1)^2);
    end
    
    [xc, yc, R] = cerchioTrePunti(P, A_fless, B_fless);
    
    theta = linspace(0,2*pi,200);
    x = xc + R*cos(theta);
    y = yc + R*sin(theta);
    
    plot(x, y, 'b-', 'LineWidth', 2);

    xlim([min([A0(1), B0(1), A(1), B(1), A_fless(1), B_fless(1), xc-R])-10, ...
      max([A0(1), B0(1), A(1), B(1), A_fless(1), B_fless(1), xc+R])+10]);

    ylim([min([A0(2), B0(2), A(2), B(2), A_fless(2), B_fless(2), yc-R])-10, ...
      max([A0(2), B0(2), A(2), B(2), A_fless(2), B_fless(2), yc+R])+10]);

    hold off
end

% Attenzione che come argomenti la funzione circonferenzaFlessi vuole dei
% vettore riga!
circonferenzaFlessi(A0',B0',A1',B1')
circonferenzaFlessi(A0',B0',A2',B2')
