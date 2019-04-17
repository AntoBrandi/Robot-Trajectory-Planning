%%  ROBOT TRAJECTORY PLANNING
% The scope of this code is to create an algorithm to set a trajectory for
% a robot interpolating linear polynomials with parabolic blends


t = chop(t,4);

for r = 1:2
    %% Fase di calcolo preeliminare 
    % Calcolo i rimanenti parametri utili all'esecuzione dell'algoritmo
    %
    % - d_tk = tk+1 - tk --> Intervallo di tempo che separa qk+1 da qk
    % - qp  --> Velocità della traiettoria che connette qk a qk+1 funzione lineare
    %        del tempo (qp_k,k+1)
    % - qpp --> Accelerazione nel tratto parabolico di durata dtp (qpp_k)


    % d_tk # INTERVALLO DI TEMPO CHE SEPARA DUE PUNTI CONSECUTIVI #
    d_tk = zeros(size(q(r,:))-1);    % Preallocazione del vettore che conterrà d_tk

    % Assegnazione della durata di ciascun intervallo di tempo che separa due
    % punti consecutivi qk e qk+1
    % La dimensione di questo vettore sarà N-1
    for i = 1:(length(q(r,:))-1)
        d_tk(i) = t(i+1)-t(i);

        % Error Management
        if d_tk(i) <=  dtp(i)
            error('dtp troppo grande! Riduci dtp o aumenta il tempo tra due punti consecutivi');
        end

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
    qp=zeros(1,(length(q(r,:))+1));  % Preallocazione del vettore che conterrà q.

    for i = 1:(length(q(r,:))+1)
        % Gestisco l'eccezione per la velocità di partenza
        if i==1
            qp(i)=0;

        % Gestisco l'eccezione per la velocità di arrivo
        elseif i==(length(q(r,:))+1)
            qp(i)=0;

        % Comportamento standard per i punti intermedi
        else
            qp(i)=(q(r,i)-q(r,i-1))/(d_tk(i-1));
        end  
    end


    % qpp # ACCELERAZIONE NEL TRATTO PARABOLICO #
    % Calcolo della accelerazione nel tratto parabolico di durata dtp
    % Ciascun elemento del vettore qpp sarà un valore della accelerazione
    % espresso come qpp_k
    qpp=zeros(1,length(q(r,:))); % Preallocazione del vettore che conterrà qpp

    for i=1:length(q(r,:))
        qpp(i)=(qp(i+1)-qp(i))/(dtp(i));
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
    linear_trajectory = interp1(t,q(r,:),time,'linear');



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

    %parabole=zeros(length(q),(dtp/accuracy)+1); %Preallocazione matrice dei raccordi

    % Per ogni punto di via, calcolo il corrispondente punto di partenza (tA,qA) 
    % e di arrivo (tB,qB) del raccordo parabolico tra il tratto lineare che precede 
    % il punto di via ed il successivo
    for j=1:length(q(r,:))

        % ECCEZIONE RACCORDO INIZIALE
        % Punto di partenza del raccordo parabolico del primo punto di via ha:
        % - ascissa = ascissa del primo punto di via qA = q(j)
        % - ordinata = ordinata del primo punto di via - dtp/2
        if j==1

            % punto iniziale
            tA = t(j)-(dtp(j)/2);
            qA = q(r,j);

            % punto finale
            tB = t(j)+(dtp(j)/2);
            qB = linear_trajectory(chop(((tB/accuracy)+1),5));


        % ECCEZIONE RACCORDO FINALE
        % Punto di arrivo del raccordo parabolico dell'ultimo punto di via ha:
        % - ascissa = ascissa dell'ultimo punto di via qB = q(j);
        % - ordinata = ordinata dell'ultimo punto di via + dtp/2
        elseif j==length(q(r,:))
            % punto iniziale
            tA = t(j)-(dtp(j)/2);
            qA = linear_trajectory(chop(((tA/accuracy)+1),5));

            %punto finale
            tB = t(j)+(dtp(j)/2);
            qB = q(r,j);


        % COMPORTAMENTO STANDARD # RACCORDO CENTRALE
        % Estraggo il punto iniziale (tA,qA) e finale (tB,qB) dalla traiettoria
        % lineare a tratti (linear_trajectory)
        else

            % punto iniziale
            tA = t(j)-(dtp(j)/2);
            qA = linear_trajectory(chop(((tA/accuracy)+1),5));

            % punto finale
            tB = t(j)+(dtp(j)/2);
            qB = linear_trajectory(chop(((tB/accuracy)+1),5));

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

        for i=1:((dtp(j)/accuracy)+1)
            parabole(j,i)= p(i);
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
    time = (t(1)-(dtp(1)/2)):accuracy:(t(length(t))+(dtp(length(dtp))/2));
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

        % TRATTO PARABOLICO
        % Istante di tempo attuale nell'intorno del punto di via di dimensione dtp/2
        if ((chop((time(i)),5)>=chop((t(j)-(dtp(j)/2)),5)) && (chop(time(i),5))<=chop((t(j)+(dtp(j)/2)),5))
            % Reinizializzo k ad ogni nuovo punto di via
            if chop(time(i),5) == chop((t(j)-(dtp(j)/2)),5)
                k=1;          
            end
            % Assegno ad y(i) il valore del tratto parabolico all'istante corrispondente 
            y(i)=parabole(j,k);

            % Incremento j alla fine di ciascun tratto parabolico
            if chop(time(i),5) == chop((t(j)+(dtp(j)/2)),5)
                j=j+1;
            end

        % TRATTO LINEARE
        % Istante di tempo attuale fuori dall'intorno del punto di via di dimensione dtp/2
        else

            % Assegno ad y(i) il valore del tratto lineare all'istante corrispondente 
            y(i)=linear_trajectory(i-((dtp(1)/2)/accuracy));
        end

        k=k+1;
        i=i+1;
    end

    %% PRESENTING THE DATA

    lt = t(1):accuracy:t(length(t));

    % Speed
    yp = diff(y);
    yp(length(y))=yp((length(y)-1));
    linear_trajectoryp=diff(linear_trajectory);
    linear_trajectoryp(length(linear_trajectory)) = linear_trajectoryp((length(linear_trajectory)-1));

    % Acceleration
    ypp = diff(yp);
    ypp(length(y))=ypp((length(y)-1));
    linear_trajectorypp=diff(linear_trajectoryp);
    linear_trajectorypp(length(linear_trajectory)) = linear_trajectorypp((length(linear_trajectory)-1));


    figure(r)
    subplot(3,1,1)
    plot(time,y,'LineWidth',2)
    hold on
    plot(t,q(r,:),'--o','MarkerSize',10,'Color','m')
    xlabel('Time - [s]'),ylabel('Position - [rad]')
    title('Posizione')
    legend('Traiettoria Parabolico-Lineare','Traiettoria Lineare a Tratti')

    subplot(3,1,2)
    plot(time,yp,'LineWidth',2)
    hold on
    plot(lt,linear_trajectoryp,'--','Color','m')
    xlabel('Time - [s]'),ylabel('Speed - [rad/s]')
    title('Velocità')
    legend('Velocità Traiettoria Parabolico-Lineare','Velocità Traiettoria Lineare a Tratti')

    subplot(3,1,3)
    plot(time,ypp,'LineWidth',2)
    ylim([-1e-04 1e-04])
    hold on
    plot(lt,linear_trajectorypp,'--','Color','m')
    xlabel('Time - [s]'),ylabel('Acceleration - [rad/s^2]')
    title('Accelerazione')
    legend('Accelerazione Traiettoria Parabolico-Lineare','Accelerazione Traiettoria Lineare a Tratti')
end
