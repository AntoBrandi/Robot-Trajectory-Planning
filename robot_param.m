%% ROBOT PARAMETERS FILE
% Dal momento che i parametri si riferiscono ad un robot planare a due
% bracci, gli unici parametri non nulli che è necessario specificare dalla
% convenzione di Denavit-Hartemberg sono Ai e Thetai.

% LUNGHEZZA DEI BRACCI
A = [1 1];

% POSA INIZIALE
% Si riferisce alle coordinate cartesiane dell'organo terminale nel punto
% iniziale
pI = [1, 1];

% POSA FINALE
% Si riferisce alle coordinate cartesiane dell'organo terminale nel punto
% finale
pF = [-1, -1];

% DURATA
% Tempo che intercorre tra la posa iniziale e quella finale
ti=0;
tf = 5;

% TRAIETTORIA SPAZIO OPERATIVO
% Realizzo la traiettoria s(t) molto semplice. Nello stesso intervallo di
% tempo percorro la stessa distanza sulla curva.
accuracy=0.01;
s=0:accuracy:1;

% PUNTI DI VIA
% Scelta del numero di punti di via da estrarre dalla traiettoria dello
% spazio operativo. Tali punti saranno scelti per semplicità equidistanti
% tra loro
n_via = 10;










