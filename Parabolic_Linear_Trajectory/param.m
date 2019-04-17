%%  PARAMETERS FILE
% File that will be loaded into the Trajectory Planning script
% This will cointain all the informations about the trajectory points
%
% VARIABILI UTILIZZATE
% q --> Insieme (Vettore) dei punti di via
% t --> Istanti di tempo di ciascun punto di via
% dtp --> Durata del Tratto Parabolico
% accuracy --> Precisione con la quale sarà rappresentata la traiettoria


%% Punti di via (qk) 
% Coordinate dei punti di via per il singolo giunto
q=[0 2*pi pi/2 pi];


%% Istanti di tempo in corrispondenza dei punti di via (d_tk)
t=[0 3 4 5];


%% Durata del tratto parabolico (d_tk')
% Sarà specificato un valore per ciascun punto di via
%dtp = [0.1 0.9 0.9 0.5];
% Oppure posso assegnare a tutti lo stesso valore
dtp = 0.3*ones(1,length(q));

%% Rappresentazione della traiettoria
accuracy = 0.001;


