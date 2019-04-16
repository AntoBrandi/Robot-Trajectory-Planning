%% TWO LINK PLANAR ARM TRAJECTORY PLANNING
% Viene usata la struttura del manipolatore planare a due bracci sul quale
% si intende pianificare una traiettoria da far seguire all'organo
% terminale del manipolatore e, mediante inversione cinematica su un certo
% set di punti, pianificare la traiettoria di ciascun giunto con
% l'algoritmo di pianificazione della traiettoria passante in prossimità
% dei punti di via.

%% Definizione del Robot
% Carico i parametri che specificano le dimensioni e la posa del
% manipolatore
run robot_param


% Definisco ciascun braccio (link) del robot attraverso la convenzione di
% Denavit-Haremberg standard) in presenza di tre giunti rotoidali (0)
L = Link([0,0.1,0.2,pi/2,0]);
% theta di ai alphai
L(1) = Link([0 0 A(1) 0],'standard');
L(2) = Link([0 0 A(2) 0],'standard');

% Creo l'oggetto @planar_robot che conterrà le informazioni del robot che
% deve essere studiato.
planar_robot = SerialLink(L,'name','planar robot');

%% Estrazione delle coordinata dell'organo terminale del manipolatore nella posa iniziale e finale
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
T = ctraj(TI,TF,s);

%% Estrazione dei punti di via
% Dalla traiettoria cartesiana estratta precedentemente estraggo un certo
% numero di punto di via specificato nel file dei parametri, che siano
% equidistanti tra loro e sui quali effettuo una operazione di inversione
% cinematica
distance=floor(length(s)/(n_via-2));

% Assegno alla matrice q l'insieme dei punti di via per ciascun giunto 
% RIGHE = giunti del manipolatore
% COLONNE = numero dei punti di via scelti
q = zeros(2,n_via); % Preallocazione della memoria
% Assegno alla matrice t gli istanti di tempo associati a ciascun punto
t = zeros(1,n_via); % Preallocazione della memoria

% Primo punto di via qI
q(1,1)=qI(1);
q(2,1)=qI(2);
t(1,1)=ti;


% Ultimo punto di via
q(1,length(q))=qF(1);
q(2,length(q))=qF(2);
t(1,length(q))=tf;

% Punti di via intermedi
i=1;
while i<=(n_via-2)
    % Inversione cinematica sul k-esimo elemento della traiettoria T
    qT = planar_robot.ikine(T(:,:,i*distance),[0 0],'mask',[1 1 0 0 0 0]);
    q(1,i+1)=qT(1);
    q(2,i+1)=qT(2);
    t(1,i+1)=(tf*(i*distance))/length(s);
    
    % Incremento i per l'interazione successiva
    i=i+1;
end

%% Traiettoria nello spazio dei giunti
% Parametri utili all'esecuzione del calcolo della traiettoria per ciascun
% giunto del manipolatore

% Durata del tratto parabolico (d_tk')
dtp = 0.2*ones(1,length(q));

% Per ciscun giunto del manipolatore calcolo la traiettoria con l'algoritmo
% di interpolazione parabolico-lineare in prossimità dei punti di via







% RAPPRESENTAZIONE DEL MANIPOLATORE PLANARE A DUE BRACCI (posa iniziale)
planar_robot.plot([qI(1) qI(2)]),axis equal




