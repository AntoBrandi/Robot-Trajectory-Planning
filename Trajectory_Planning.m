%%  ROBOT TRAJECTORY PLANNING
% The scope of this code is to create an algorithm to set a trajectory for
% a robot interpolating linear polynomials with parabolic blends


%% Parameters
% Conterrà tutte le informazioni sulla traiettoria
% - qk
% - d_tk
% - d_tk'

run param
debug =0;


%% Fase di calcolo preeliminare 
% Calcolo i rimanenti parametri utili all'esecuzione dell'algoritmo

% TEMPO CHE SEPARA DUE PUNTI CONSECUTIVI d_tk
d_tk = zeros(size(q)-1);    % Preallocazione del vettore che conterrà d_tk

% Assegno le variabili attuali al vettore d_tk
% Il calcolo è riferito soltanto ai primi N-1 punti del vettore
for i = 1:(length(q)-1)
    d_tk(i) = t(i+1)-t(i);
end


% VELOCITA' TRA DUE PUNTI CONSECUTIVI q.
% Calcolo della velocità del tratto lineare della traiettoria
% Ciascun elemento del vettore qp sarà un valore della velocità espresso
% come q._k-1,k
qp=zeros(1,(length(q)+1));  % Preallocazione del vettore che conterrà q.

for i = 1:(length(q)+1)
    % Gestisco l'eccezione per la velocità di partenza
    if i==1
        qp(i)=0;
        
    % Gestisco l'eccezione per la velocità di arrivo
    elseif i==(length(q)+1)
        qp(i)=0;
        
    % Comportamento standard per i punti intermedi
    else
        qp(i)=(q(i)-q(i-1))/(d_tk(i-1));
    end  
end


% ACCELERAZIONE NEL TRATTO PARABOLICO q..
% Calcolo della accelerazione nel tratto parabolico
% Ciascun elemento del vettore qpp sarà un valore della accelerazione
% espresso come q..k

% Preallocazione del vettore che conterrà q..
qpp=zeros(1,length(q));
for i=1:length(q)
    qpp(i)=(qp(i+1)-qp(i))/(dtp);
end



%% Interpolazione Lineare dei Punti di Via
% Rappresentazione della traiettoria lineare a tratti passante per i punti
% di via. Tale traiettoria sarà conservata nel vettore @linear_trajectory
time = t(1):accuracy:t(length(t));
linear_trajectory = interp1(t,q,time);
figure(1)
plot(time,linear_trajectory)
xlabel('Time - t [s]'),ylabel('Parameters - q [rad]')
title('Interpolazione Lineare a Tratti per i Punti di Via')



%% Raccordo Parabolico in prossimità dei punti di via
% Calcolo l'equazione della parabola in prossimità di ciascun punto di via

% Preallocazione della matrice di parametri che conterrà i punti iniziale e
% finale del tratto parabolico
parabole=zeros(length(q),(dtp/accuracy)+1);

% Calcolo in prossimità di ciascun punto di via l'equazione di una parabola
% che raccorderà due tratti lineari della caratteristica. Ciascuna di
% queste parabole sarà una riga della matrice parabole
for j=1:length(q)
    if debug == 1
        fprintf('________________________________________\n');
        fprintf('-- PARABOLIC TRAJECTORY CREATION j=%d \n',j);
    end
    
    % Eccezione Tratto Iniziale
    if j==1

        % punto iniziale
        tA = t(j)-(dtp/2);
        qA = q(j);
        
        % punto finale
        tB = t(j)+(dtp/2);
        qB = linear_trajectory(chop(((tB/accuracy)+1),5));
        
        if debug==1
            fprintf('Parabola Iniziale\n');
            fprintf('qa=%d,ta=%d,qb=%d,tb=%d\n',qA,tA,qB,tB);
        end
        
    %Eccezione Tratto Finale
    elseif j==length(q)
        % punto iniziale
        tA = t(j)-(dtp/2);
        qA = linear_trajectory(chop(((tA/accuracy)+1),5));
        
        %punto finale
        tB = t(j)+(dtp/2);
        qB = q(j);
        
        if debug==1
            fprintf('Parabola Finale\n');
            fprintf('qa=%d,ta=%d,qb=%d,tb=%d\n',qA,tA,qB,tB);
        end
        
    % Comportamento Standard - Tratto Centrale
    else
        if debug==1
            fprintf('Parabola Centrale\n');
        end
            
        % punto iniziale
        tA = t(j)-(dtp/2);
        qA = linear_trajectory(chop(((tA/accuracy)+1),5));
        
        % punto finale
        tB = t(j)+(dtp/2);
        qB = linear_trajectory(chop(((tB/accuracy)+1),5));
        
        if debug==1
            fprintf('qa=%d,ta=%d,qb=%d,tb=%d\n',qA,tA,qB,tB);
        end
    end
    
    % Calcolo dei Coefficienti del tratto parabolico
    a2 = qpp(j)/2;
    a1 = (qA - qB + (a2*(tB^2)) - (a2*(tA^2)))/(tA-tB);
    a0 = qB - (a2*(tB^2)) - (a1*tB);
    
    % Estrazione della Parabola
    time = tA:accuracy:tB;
    p = a0 + a1*time + a2*(time.^2);
    
    for i=1:((dtp/accuracy)+1)
        parabole(j,i)= p(i);
    end
    
    if debug==1
        figure(j)
        plot(time,p);
    end
end


%% Composizione del tratto Lineare e Parabolico
% Intervallo di tempo più lungo rispetto alla durata standard
time = (t(1)-(dtp/2)):accuracy:(t(length(t))+(dtp/2));
% Vettore che conterrà la traiettoria finale
y = zeros(1,length(time));

j=1;
k=1;
i=1;
while i<=length(time)
    if debug == 2
        fprintf('-------------------------------------------\n');
        fprintf('-- NEW LOOP i=%d, time(i)=%d --\n',i,time(i));
        fprintf('-- t(j)-(dtp/2)=%d -- t(j)+(dtp/2)=%d \n',(t(j)-(dtp/2)),(t(j)+(dtp/2)))
    end
    
    
    if ((chop((time(i)),5)>=chop((t(j)-(dtp/2)),5)) && (chop(time(i),5))<=chop((t(j)+(dtp/2)),5))
        if debug == 2
            fprintf('Parabolic Loop for j = %d and i= %d \n',j,i);
        end
        % Reinizializzo k ad ogni iterazione
        if chop(time(i),5) == chop((t(j)-(dtp/2)),5)
            k=1;
            if debug == 2
               fprintf('Resetting k = %d \n',k);
            end
            
        end
        
        % y Tratto Parabolico 
        if debug == 2
           fprintf('Setting Parabolic y value for indeces i = %d, j=%d, k=%d \n',i,j,k);
        end
        y(i)=parabole(j,k);
        
        % Incremento j solo una volta alla fine di ciascun tratto
        if chop(time(i),5) == chop((t(j)+(dtp/2)),5)
            j=j+1;
            if debug == 2
               fprintf('Incrementing j = %d \n',j);
            end
        end
    else
        % y Tratto lineare
        if debug == 2
           fprintf('Setting Linear y value for index i = %d \n',i);
        end
        y(i)=linear_trajectory(i-((dtp/2)/accuracy));
    end
    
    k=k+1;
    i=i+1;
end

%% PRESENTING THE DATA
figure(5)
plot(time,y,'LineWidth',2)
hold on
plot(t,q,'--o','MarkerSize',10,'Color','m')
xlabel('Time - t [s]'),ylabel('Parameters - q [rad]')
title('Interpolazione Parabolico Lineare in prossimità dei punti di via')
legend('Traiettoria Parabolico-Lineare','Traiettoria Lineare a Tratti')



