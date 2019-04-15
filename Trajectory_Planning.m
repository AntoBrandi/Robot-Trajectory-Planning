%%  ROBOT TRAJECTORY PLANNING
% The scope of this code is to create an algorithm to set a trajectory for
% a robot interpolating linear polynomials with parabolic blends


%% Load Parameters
% Conterrà tutte le informazioni sulla traiettoria
% - qk
% - d_tk
% - dtp

run param


%% Fase di calcolo preeliminare 
% Calcolo i rimanenti parametri utili all'esecuzione dell'algoritmo
%
% - d_tk = tk+1 - tk --> Intervallo di tempo che separa qk+1 da qk
% - qp  --> Velocità della traiettoria che connette qk a qk+1 funzione lineare
%        del tempo (qp_k,k+1)
% - qpp --> Accelerazione nel tratto parabolico di durata dtp (qpp_k)


% d_tk # INTERVALLO DI TEMPO CHE SEPARA DUE PUNTI CONSECUTIVI #
d_tk = zeros(size(q)-1);    % Preallocazione del vettore che conterrà d_tk

% Assegnazione della durata di ciascun intervallo di tempo che separa due
% punti consecutivi qk e qk+1
% La dimensione di questo vettore sarà N-1
for i = 1:(length(q)-1)
    d_tk(i) = t(i+1)-t(i);
end


% qp # VELOCITA' TRA DUE PUNTI CONSECUTIVI #
% Calcolo della velocità nel tratto lineare della traiettoria che separa
% due punti consecutivi qk-1 e qk
% Ciascun elemento del vettore qp sarà un valore della velocità espresso
% come qp_k-1,k
%
% GESTIONE ECCEZIONI
% Si consideri la velocità del tratto iniziale qp_0,1 = 0
% Si consideri la velocità del tratto finale qp_N,N+1 = 0
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


% qpp # ACCELERAZIONE NEL TRATTO PARABOLICO #
% Calcolo della accelerazione nel tratto parabolico di durata dtp
% Ciascun elemento del vettore qpp sarà un valore della accelerazione
% espresso come qpp_k
qpp=zeros(1,length(q)); % Preallocazione del vettore che conterrà qpp

for i=1:length(q)
    qpp(i)=(qp(i+1)-qp(i))/(dtp);
end



%% Interpolazione Lineare dei Punti di Via
% Rappresentazione della traiettoria lineare a tratti passante per i punti
% di via. Tale traiettoria sarà conservata nel vettore @linear_trajectory
% Tale vettore rappresentativo della traiettoria sarà poi interrogato nella
% fase di creazione della traiettoria parabolica per l'estrazione dei punti
% di partenza e di arrivo del raccordo parabolico in prossimità dei punti
% di via.

% L'accuratezza della traiettoria sarà descritta da @accuracy
time = t(1):accuracy:t(length(t));
% Sfrutto la funzione interp1 per l'interpolazione lineare dei dati q,t nei
% punti descritti da time.
linear_trajectory = interp1(t,q,time,'linear');

if debug==1
    figure(1)
    plot(time,linear_trajectory)
    xlabel('Time - t [s]'),ylabel('Parameters - q [rad]')
    title('Interpolazione Lineare a Tratti per i Punti di Via')
end



%% Raccordo Parabolico in prossimità dei punti di via
% Calcolo l'equazione della parabola in prossimità di ciascun punto di via
% In corrispondenza di ciascun punto di via sono calcolate le equazioni del
% tratto parabolico che raccorda il tratto lineare precedente al punto di
% via scelto con il tratto lineare successivo al punto di via scelto, nota:
% - la durata del tratto parabolico (dtp)
% - la accelerazione del tratto parabolico (qpp)
% - il punto iniziale e finale del tratto parabolico (linear_trajectory --> tA,qA,tB,qB)
%
% Ciascuno di questi raccordi parabolici sarà conservato come una riga
% della matrice @parabole che avrà quindi tante righe quanti sono i punti
% di via (N) e tante colonne in funzione della durata del tratto parabolico
% (dtp) e della accuratezza con la quale lo si vuole rappresentare
% (accuracy)
%
% GESTIONE ECCEZIONI
% Si consideri che il punto di partenza del primo raccordo parabolico
% precede l'inizio della traiettoria < t(1) e che il punto di arrivo
% dell'ultimo raccordo parabolico segue la fine della traiettoria > t(length(t))
% Pertanto, la traiettoria finale parabolico lineare, avrà una durata
% maggiore di un fattore [t_N - t_1 + (dtp_1 + dtp_N)/2]

parabole=zeros(length(q),(dtp/accuracy)+1); %Preallocazione matrice dei raccordi

% Per ogni punto di via, calcolo il corrispondente punto di partenza (tA,qA) 
% e di arrivo (tB,qB) del raccordo parabolico tra il tratto lineare che precede 
% il punto di via ed il successivo
for j=1:length(q)
    if debug == 1
        fprintf('________________________________________\n');
        fprintf('-- PARABOLIC TRAJECTORY CREATION j=%d \n',j);
    end
    
    % ECCEZIONE RACCORDO INIZIALE
    % Punto di partenza del raccordo parabolico del primo punto di via ha:
    % - ascissa = ascissa del primo punto di via qA = q(j)
    % - ordinata = ordinata del primo punto di via - dtp/2
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
        
    % ECCEZIONE RACCORDO FINALE
    % Punto di arrivo del raccordo parabolico dell'ultimo punto di via ha:
    % - ascissa = ascissa dell'ultimo punto di via qB = q(j);
    % - ordinata = ordinata dell'ultimo punto di via + dtp/2
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
        
    % COMPORTAMENTO STANDARD # RACCORDO CENTRALE
    % Estraggo il punto iniziale (tA,qA) e finale (tB,qB) dalla traiettoria
    % lineare a tratti (linear_trajectory)
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
    
    % CALCOLO DEI COEFFICIENTI DELL'EQUAZIONE DEL RACCORDO PARABOLICO
    % Dalla conoscenza di :
    % - punto iniziale (tA,qA)
    % - punto finale (tB,qB)
    % - accelerazione (qpp)
    % calcolo i coefficienti dell'equazione della parabola (a0,a1,a2)
    % con la risoluzione di un sistema di 3 equazioni in 3 incognite
    %       SISTEMA DA RISOLVERE
    %  __   
    % |qA = a0 + a1*tA + a2*tA^2
    % |qB = a0 + a1*tB + a2*tB^2
    % |qpp_k = 2*a2
    % |__
    A = [1 tA (tA^2);1 tB (tB^2);0 0 2];
    B = [qA qB qpp(j)];
    C = inv(A);
    C = C';
    X = B*C;
    
    % Equazione del raccordo Parabolico
    time = tA:accuracy:tB;
    p = X(1) + X(2)*time + X(3)*(time.^2);
    
    for i=1:((dtp/accuracy)+1)
        parabole(j,i)= p(i);
    end
    
    if debug==1
        figure(j)
        plot(time,p);
        title('Raccordo Parabolico %d esimo',j);
        xlabel('Time - t [s]'),ylabel('Parameters - q [rad]')
    end
end


%% Composizione del tratto Lineare e Parabolico
% Si va ora a comporre la traiettoria parabolico lineare come aggregazione
% di tratti in corrispondenza dei quali la traiettoria è parabolica e
% tratti nei quali la traiettoria è lineare.
% 
% Per ogni istante di tempo definito dalla accuracy della traiettoria,
% decido se assegnare alla traiettoria finale parabolico-lineare (y) un
% valore della traiettoria lineare (linear_trajectory) oppure il valore di
% una delle traiettorie paraboliche (parabole)
%
% Si consideri anche che la durata della nuova traiettoria
% paraboilico-lineare (y) sarà maggiore rispetto alla traiettoria lineare
% di un fattore [t_N - t_1 + (dtp_1 + dtp_N)/2]
time = (t(1)-(dtp/2)):accuracy:(t(length(t))+(dtp/2));
% Vettore che conterrà la traiettoria finale parabolico-lineare
y = zeros(1,length(time));



% Analizzo la traiettoria istante per istante (definito dalla accuracy)
% - SE istante di tempo attuale si trova in un intorno di un punto di via
%      dalla dimensione <= dtp/2
%   ALLORA il tratto sarà parabolico y(i) = parabole(j,k)
% - SE istante di tempo attuale di trova al di fuori di un intorno di uno
%      dei punti di via dalla dimensione <= dtp/2
%   ALLORA il tratto sarà lineare linear_trajectory(i-((dtp/2)/accuracy))
%
% i --> Contatore istanti di tempo per tutta la traiettoria y(t)
% j --> Contatore punti di via. Ogni volta che concludo il raccordo
%       parabolico in prossimità di un punto di via, incremento j
% k --> Contatore istanti di tempo per il solo raccordo parabolico j-esimo
%       sarà reinizializzato ad 1 ad ogni nuovo raccordo
i=1;j=1;k=1;
while i<=length(time)
    if debug == 2
        fprintf('-------------------------------------------\n');
        fprintf('-- NEW LOOP i=%d, time(i)=%d --\n',i,time(i));
        fprintf('-- t(j)-(dtp/2)=%d -- t(j)+(dtp/2)=%d \n',(t(j)-(dtp/2)),(t(j)+(dtp/2)))
    end
    
    % TRATTO PARABOLICO
    % Istante di tempo attuale nell'intorno del punto di via di dimensione dtp/2
    if ((chop((time(i)),5)>=chop((t(j)-(dtp/2)),5)) && (chop(time(i),5))<=chop((t(j)+(dtp/2)),5))
        if debug == 2
            fprintf('Parabolic Loop for j = %d and i= %d \n',j,i);
        end
        % Reinizializzo k ad ogni nuovo punto di via
        if chop(time(i),5) == chop((t(j)-(dtp/2)),5)
            k=1;
            if debug == 2
               fprintf('Resetting k = %d \n',k);
            end
            
        end
        
        if debug == 2
           fprintf('Setting Parabolic y value for indeces i = %d, j=%d, k=%d \n',i,j,k);
        end
        % Assegno ad y(i) il valore del tratto parabolico all'istante corrispondente 
        y(i)=parabole(j,k);
        
        % Incremento j alla fine di ciascun tratto parabolico
        if chop(time(i),5) == chop((t(j)+(dtp/2)),5)
            j=j+1;
            if debug == 2
               fprintf('Incrementing j = %d \n',j);
            end
        end
    
    % TRATTO LINEARE
    % Istante di tempo attuale fuori dall'intorno del punto di via di dimensione dtp/2
    else
        
        if debug == 2
           fprintf('Setting Linear y value for index i = %d \n',i);
        end
        % Assegno ad y(i) il valore del tratto lineare all'istante corrispondente 
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



