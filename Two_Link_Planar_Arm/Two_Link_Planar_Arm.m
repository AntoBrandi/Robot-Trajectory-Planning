%% TWO LINK PLANAR ARM TRAJECTORY PLANNING
% Viene usata la struttura del manipolatore planare a due bracci sul quale
% si intende pianificare una traiettoria da far seguire all'organo
% terminale del manipolatore e, mediante inversione cinematica su un certo
% set di punti, pianificare la traiettoria di ciascun giunto con
% l'algoritmo di pianificazione della traiettoria passante in prossimità
% dei punti di via.
clear all

%% Definizione del Robot
% Carico i parametri che specificano le dimensioni e la posa del
% manipolatore
run robot_param


% Definisco ciascun braccio (link) del robot attraverso la convenzione di
% Denavit-Haremberg standard) in presenza di due giunti rotoidali (0)
L = Link([0,0.1,0.2,pi/2,0]);
% theta di ai alphai
L(1) = Link([0 0 a(1) 0],'standard');
L(2) = Link([0 0 a(2) 0],'standard');

% Creo l'oggetto @planar_robot che conterrà le informazioni del robot che
% deve essere studiato.
planar_robot = SerialLink(L,'name','planar robot');

%% Estrazione delle coordinate dell'organo terminale del manipolatore nella posa iniziale e finale
% POSA INIZIALE
% Matrice di trasformazione omogenea riferita alla posa iniziale del robot
TI = transl(pI(1), pI(2), 0);
% Vettore dei parametri di giunto che realizzano la posa iniziale
qI = planar_robot.ikine(TI,[0 0],'mask',[1 1 0 0 0 0]);


% POSA FINALE 
% Matrice di trasformazione omogenea riferita alla posa finale del robot
TF = transl(pF(1),pF(2),0);
% Vettore dei parametri di giunto che realizzano la posa finale
qF = planar_robot.ikine(TF,[0 0],'mask',[1 1 0 0 0 0]);


%% Generazione della traiettoria cartesiana
% Matrice di trasformazione omogenea estratta istante per istante
% In T sarà contenuta una traiettoria nello spazio operativo, ovvero una
% sequenza di pose nelle quali l'organo terminale del manipolatore debba
% trovarsi
% E' stata utilizzata una equazione polinomiale per definire l'ascissa
% curvilinea s in funzione del tempo t
T = ctraj(TI,TF,chop(s,6));

%% Estrazione dei punti di via
% Dalla traiettoria cartesiana estratta precedentemente estraggo un certo
% numero di punti di via specificato nel file dei parametri, che siano
% equidistanti tra loro e sui quali effettuo una operazione di inversione
% cinematica
% distance = distanza tra due punti di via consecutivi. Sto supponendo che
% siano equidistanti i punti di via estratti
distance=floor(length(s)/(n_via-2));

% Assegno alla matrice q l'insieme dei punti di via per ciascun giunto 
% RIGHE = giunti del manipolatore
% COLONNE = numero dei punti di via scelti
q = zeros(planar_robot.n,n_via); % Preallocazione della memoria
% Assegno alla matrice t gli istanti di tempo associati a ciascun punto
t = zeros(1,n_via); % Preallocazione della memoria

% ECCEZIONE Primo punto di via qI
q(1,1)=qI(1);
q(2,1)=qI(2);
t(1,1)=ti;

% ECCEZIONE Ultimo punto di via qF
q(1,length(q))=qF(1);
q(2,length(q))=qF(2);
t(1,length(q))=tf;

% COMPORTAMENTO STANDARD Punti di via intermedi
% In corrispondenza dell'istante al quale ho disposto i punti di via,
% estraggo la posa dell'organo terminale in quell'istante dal vettore delle
% pose T e dalla specifica matrice di trasformazione omogenea nello
% specifico istante, effettuo l'inversione cinematica per conoscere in
% quell'istante i valori dei parametri di giunto che ne garantiscano la
% posa
i=1;
while i<=(n_via-2)
    % Inversione cinematica sul k-esimo elemento della traiettoria T
    qT = planar_robot.ikine(T(:,:,i*distance),[0 0],'mask',[1 1 0 0 0 0]);
    q(1,i+1)=qT(1);
    q(2,i+1)=qT(2);
    % Uso una semplice proporzione per la determinazione del vettore t
    t(1,i+1)=t(1,i)+(tf/(n_via-1));
    
    % Incremento i per l'interazione successiva
    i=i+1;
end

%% Traiettoria nello spazio dei giunti
% Per ciscun giunto del manipolatore calcolo la traiettoria con l'algoritmo
% di interpolazione parabolico-lineare in prossimità dei punti di via
% Essendo un robot planare a due bracci ho solo due giunti
% Calcolo la traiettoria di ciascun giunto

run Trajectory_Planning_Adapted


%% Animazione del manipolatore 
% Rappresentazione della animazione del planar_robot facendo seguire a
% ciascun giunto la traiettoria che è stata calcolata dall'algoritmo di
% pianificazione
figure(3)
planar_robot.plot(Q,'delay',1e-10);








