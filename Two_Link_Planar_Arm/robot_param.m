%% ROBOT PARAMETERS FILE
% Specifico i parametri utili per il dimensionamento del robot, della sua
% posa e della sua traiettoria tra la posa iniziale e quella finale

% LUNGHEZZA DEI BRACCI
% Si faccia sempre riferimento al caso di un manipolatore planare a due
% bracci
a = [1 1];

% POSA INIZIALE
% Si riferisce alle coordinate cartesiane dell'organo terminale nel punto
% iniziale
pI = [1.5, 1.3];

% POSA FINALE
% Si riferisce alle coordinate cartesiane dell'organo terminale nel punto
% finale
pF = [1.5, -1.3];

% DURATA
% Tempo che intercorre tra la posa iniziale e quella finale
ti=0;
tf = 5;

% TRAIETTORIA SPAZIO OPERATIVO
% Realizzo la traiettoria s(t) molto semplice. Nello stesso intervallo di
% tempo percorro la stessa distanza sulla curva.
accuracy=0.001;
% Realizzo una traiettoria polinomiale del III ordine che leghi l'ascissa
% curvilinea s al tempo t. In questo modo posso assegnare un valore
% iniziale e finale alla ascissa curvilinea [0 e 1] ed altresì assegnare un
% valore alla velocità iniziale e finale
t = ti:accuracy:tf;
% Risolvo il sistema di 4 equazioni in 4 incognite per la determinazione
% dei parametri del polinomio
A = [1 ti (ti^2) (ti^3);
     1 tf (tf^2) (tf^3);
     0 1 2*ti 3*(ti^2);
     0 1 2*tf 3*(tf^2)];
B = [0 1 0 0];
C = inv(A);
C = C';
X = B*C;

% Ascissa curvilinea con traiettoria polinomiale
s=X(1) + X(2)*t + X(3)*(t.^2) + X(4)*(t.^3);

% PUNTI DI VIA
% Scelta del numero di punti di via da estrarre dalla traiettoria dello
% spazio operativo. Tali punti saranno scelti per semplicità equidistanti
% tra loro
n_via = 10;










