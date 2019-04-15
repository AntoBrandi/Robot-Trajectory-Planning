%%  PARAMETERS FILE
% File that will be loaded into the Trajectory Planning script
% This will cointain all the informations about the trajectory points
%
% VARIABILI UTILIZZATE
% q --> Insieme (Vettore) dei punti di via
% t --> Istanti di tempo di ciascun punto di via
% dtp --> Durata del Tratto Parabolico
% accuracy --> Precisione con la quale sarà rappresentata la traiettoria
% debug --> Se >= 0 stampa nella Command Window informazioni per il debug


%% Punti di via (qk) 
% OSS. ciascun punto di via dovrebbe essere un vettore di parametri e non
% uno scalare.
q=[0 2*pi pi/2 pi];


%% Istanti di tempo in corrispondenza dei punti di via (d_tk)
t=[0 2 3 5];


%% Durata del tratto parabolico (d_tk')
% Sarà utilizzato lo stesso valore per ogni punto
dtp = 0.6;

%% Rappresentazione della traiettoria
accuracy = 0.001;
debug =0;

