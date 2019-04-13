%%  PARAMETERS FILE   %%
% File that will be loaded into the Trajectory Planning script
% This will cointain all the informations about the trajectory points


%% Punti di via (qk) 
% OSS. ciascun punto di via dovrebbe essere un vettore di parametri e non
% uno scalare.
q=[0 2*pi pi/2 pi];


%% Istanti di tempo in corrispondenza dei punti di via (d_tk)
t=[0 2 3 5];


%% Durata del tratto parabolico (d_tk')
% sarà utilizzato lo stesso valore per ogni punto
dtp = 0.2;
% dtp=0.6;

accuracy = 0.1;

