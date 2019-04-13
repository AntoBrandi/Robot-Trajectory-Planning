%%  ROBOT TRAJECTORY PLANNING

debug =1;
%% Includo il file dei parametri
% Conterrà tutte le informazioni sulla traiettoria
% - qk
% - d_tk
% - d_tk'

run param

%% Fase di calcolo preeliminare 
% Calcolo i rimanenti parametri utili all'esecuzione dell'algoritmo

% ?tk
% Preallocazione del vettore che conterrà d_tk
d_tk = zeros(size(q)-1);
% Assegno le variabili attuali al vettore d_tk
% Il calcolo è riferito soltanto ai primi N-1 punti del vettore
for i = 1:(length(q)-1)
    d_tk(i) = t(i+1)-t(i);
end

% q.
% Calcolo della velocità del tratto lineare della traiettoria
% Ciascun elemento del vettore qp sarà un valore della velocità espresso
% come q._k-1,k

% Preallocazione del vettore che conterrà q.
qp=zeros(1,(length(q)+1));
for i = 1:(length(q)+1)
    % Gestisco l'eccezione per la velocità di partenza
    if i==1
        qp(i)=0;
    % Gestisco l'eccezione per la velocità di arrivo
    elseif i==(length(q)+1)
        qp(i)=0;
    else
        qp(i)=(q(i)-q(i-1))/(d_tk(i-1));
    end  
end

% q..
% Calcolo della accelerazione nel tratto parabolico
% Ciascun elemento del vettore qpp sarà un valore della accelerazione
% espresso come q..k

% Preallocazione del vettore che conterrà q..
qpp=zeros(1,length(q));
for i=1:length(q)
    qpp(i)=(qp(i+1)-qp(i))/(dtp);
end


%% Interpolazione Lineare dei Punti di Via
time = t(1):accuracy:t(length(t));
linear_trajectory = interp1(t,q,time);
figure(1)
plot(time,linear_trajectory)
xlabel('Time - t [s]'),ylabel('Parameters - q [rad]')
title('Interpolazione Lineare a Tratti per i Punti di Via')


%% Raccordo Parabolico in prossimità dei punti di via
% Calcolo l'equazione della parabola in corrispondenza di ciascun punto di
% via
% Estrazione dei punti di partenza e di arrivo della parabola
% Preallocazione della matrice di parametri che conterrà i punti iniziale e
% finale del tratto parabolico
parabola=zeros(length(q),(dtp/accuracy)+1);

for j=1:length(q)
    % Eccezione Tratto Iniziale
    if j==1
        % punto iniziale
        tA = t(j)-(dtp/2);
        qA = q(j);
        
        % punto finale
        tB = t(j)+(dtp/2);
        qB = linear_trajectory((tB/accuracy)+1);
        
    %Eccezione Tratto Finale
    elseif j==length(q)
        %punto iniziale
        tA = t(j)-(dtp/2);
        qA = linear_trajectory((tA/accuracy)+1);
        
        %punto finale
        tB = t(j)+(dtp/2);
        qB = q(j);
        
    % Comportamento Standard
    else
        % punto iniziale
        tA = t(j)-(dtp/2);
        qA = linear_trajectory((tA/accuracy)+1);
        
        % punto finale
        tB = t(j)+(dtp/2);
        qB = linear_trajectory((tB/accuracy)+1);
    end
    
    % Calcolo dei Coefficienti del tratto parabolico
    a2 = qpp(j)/2;
    a1 = (qA - qB + (a2*(tB^2)) - (a2*(tA^2)))/(tA-tB);
    a0 = qB - (a2*(tB^2)) - (a1*tB);
    
    % Estrazione della Parabola
    time = tA:accuracy:tB;
    y = a0 + a1*time + a2*(time.^2);
    figure(2)
    plot(time,y);
       
    
    
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
    if debug == 1
        fprintf('-------------------------------------------\n');
        fprintf('-- NEW LOOP i=%d, time(i)=%d --\n',i,time(i));
        fprintf('-- t(j)-(dtp/2)=%d -- t(j)+(dtp/2)=%d \n',(t(j)-(dtp/2)),(t(j)+(dtp/2)))
    end
    
    
    if ((chop((time(i)),3)>=chop((t(j)-(dtp/2)),3)) && (chop(time(i),3))<=chop((t(j)+(dtp/2)),3))
        if debug == 1
            fprintf('Parabolic Loop for j = %d and i= %d \n',j,i);
        end
        % Reinizializzo k ad ogni iterazione
        if chop(time(i),3) == chop((t(j)-(dtp/2)),3)
            k=1;
            if debug == 1
               fprintf('Resetting k = %d \n',k);
            end
            
        end
        
        % y Tratto Parabolico 
        if debug == 1
           fprintf('Setting Parabolic y value for indeces i = %d, j=%d, k=%d \n',i,j,k);
        end
        y(i)=parabola(j,k);
        
        % Incremento j solo una volta alla fine di ciascun tratto
        if chop(time(i),3) == chop((t(j)+(dtp/2)),3)
            j=j+1;
            if debug == 1
               fprintf('Incrementing j = %d \n',j);
            end
        end
    else
        % y Tratto lineare
        if debug == 1
           fprintf('Setting Linear y value for index i = %d \n',i);
        end
        y(i)=linear_trajectory(i);
    end
    
    k=k+1;
    i=i+1;
end

plot(time,y)



