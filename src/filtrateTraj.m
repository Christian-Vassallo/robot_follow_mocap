function filteredTraj = filtrateTraj(traj, f, flag)

% 20070717
% Cette fonction retourne un tableau de trajectoire filtrees par un
% filtrage passe-bas.
% ! Attention !
% Indiquer :
%          la frequence de coupure (flag 1 ou 2)
%                   ou
%          la puissance de lissage (flag 3).
%
% Entrees :
%       - traj : tableau des trajectoires a filtrer.
%
%       - f : parametre de filtrage (float)
%               + pour flag 0 a 2 :
%                     frequence de coupure donnee en ratio de la frequence
%                     d'acquisition (float), exemple : 
%                     fcoupure = 1/3  correspond a 10 Hz pour une
%                     acquisition a 60 Hz (vicon370).
%
%               + pour flag 3 :
%                     parametre de lissage compris entre 0 et 1
%                     correspondant a une puissance de lissage de moins en
%                     moins forte (de plus en plus pres de la trajectoire a
%                     filtrer, valeur par defaut = 0.01.
%
%       - Flag : option de filtrage (int)
%                   Si flag = 0 : pas de filtrage
%                   Si flag = 1 : filtrage de Butterworth de profondeur 2
%                   Si flag = 2 : filtrage avec decomposition de Fourier
%                   Si flag = 3 : filtrage par spline (csaps)


N = length(traj);
% On extrapole les deux extremites par miroir
puiss2 = 2^(ceil(log2(N)))*2;
ajout = floor((puiss2 - N)/2);
% rajouter > ou = !!! C'est pour cela que a bug de temps a autre.
if ajout >= N 
    puiss2 = 2^(ceil(log2(N)));
    ajout = floor((puiss2 - N)/2);
end;

for i=1:size(traj,2)
    ajout_gauche = 2*traj(1,i) - traj(2:ajout+1,i);
    ajout_droite = 2*traj(N,i) - traj(N-1:-1:N-ajout+rem(N+1,2)-1,i);
    traj_miroir(:,i) = [ajout_gauche(ajout:-1:1); traj(:,i); ajout_droite];
end;

% On filtre selon l'option de filtrage choisie
switch flag 
    case 0
        filteredTraj = traj;
        
    case 1
        for i=1:size(traj,2)
            [num, den] = butter(2,f);
            outf = filtfilt(num,den,traj_miroir(:,i));
            filteredTraj(:,i) = outf(ajout+1:ajout+N);
        end;
    case 2
        for i=1:size(traj,2)
            transform = fft(traj_miroir(:,i));
            fc = floor(f * puiss2/2);
            transform_tronque = [transform(1:fc,:); zeros(puiss2-2*fc,1);...
                    transform(puiss2-fc+1:puiss2,:)];
            % transform_tronque = [transform(1:fc,:); zeros(puiss2-fc,1)];
            outf = real(ifft(transform_tronque));
            filteredTraj(:,i) = outf(ajout+1:ajout+N);
        end;
        
    case 3        
        if f >= 1 | f <= 0, 
            f = 0.01;
        end;
        for i=1:size(traj,2)
            I = find(traj(:,i));
            filteredTraj(:,i) = csaps(I,traj(I,i), f, 1:size(traj,1))';
        end;
        
    otherwise
        displayEcho(2, 'filtrateTraj.m', sprintf('l''option %d du filtrage n''existe pas !!',flag));
        filteredTraj = traj;
end;