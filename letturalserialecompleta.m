%clc; clear; close all;

% Inizializzazione variabili
T = 60*10;                      % Durata totale dell'acquisizione in secondi
deltaTs = 0.02;              % Intervallo di campionamento (10 ms)
N_campioni = T / deltaTs;    % Numero totale di campioni
dati = zeros(N_campioni, 18); % Preallocazione matrice per dati di interesse 
figure()

% Inizializzazione grafica


misurato1 = animatedline('Color','b','LineWidth',2,'DisplayName', 'angle_steer');  % Linea animata per il primo dato
misurato2 = animatedline('Color','b','LineWidth',2,'DisplayName', 'desired_filtered_torque');  % Linea animata per il secondo dato
%misurato2 = animatedline('Color','b','LineWidth',1);  % Linea animata per il secondo dato
misurato3 = animatedline('Color','g','LineWidth',2,'DisplayName', 'desired_speed_metsec');  % Linea animata per il terzo dato
misurato4 = animatedline('Color','y','LineWidth',2,'DisplayName', 'roll');  % Linea animata per il quarto dato
misurato5 = animatedline('Color','r','LineWidth',2,'DisplayName', 'speed_metsec');  % Linea animata per il primo dato
misurato6 = animatedline('Color','r','LineWidth',2,'DisplayName', 'torque');  % Linea animata per il secondo dato
%misurato6 = animatedline('Color','r','LineWidth',1);  % Linea animata per il secondo dato
misurato7 = animatedline('Color','g','LineWidth',2,'DisplayName', 'u_back_wheel');  % Linea animata per il terzo dato
misurato8 = animatedline('Color','y','LineWidth',2,'DisplayName', 'u_front_wheel');  % Linea animata per il quarto dato
misurato9 = animatedline('Color','b','LineWidth',2,'DisplayName', 'tempo');  % Linea animata per il primo dato
misurato10 = animatedline('Color','r','LineWidth',2);  % Linea animata per il secondo dato
misurato11 = animatedline('Color','g','LineWidth',2);  % Linea animata per il secondo dato
misurato12 = animatedline('Color','r','LineWidth',2);  % Linea animata per il secondo dato

ax = gca;
xlabel('Tempo [s]');
ylabel('Valori');
%legend();
title('Visualizzazione dei dati selezionati');
%ax.YLim = [-0.004 0.0049]; % Imposta i limiti dell'asse Y
%ax.YLim = [-20 15]; % Imposta i limiti dell'asse Y

grid on;

try 
    delete(s); 
    clear s;
end
% Apertura porta seriale
s = serialport("COM5", 115200, 'DataBits', 8, 'StopBits', 1);
contatore = 0;


% Creazione di un nome file basato su data e ora attuale
timestamp = datestr(now, 'mmdd_HHMMSS');  % Formattazione del timestamp (anno, mese, giorno, ora, minuti, secondi)
file_name = ['dati_acquisiti_', timestamp, '.txt'];  % Costruisce il nome del file
disp(['Salvataggio dati nel file: ', file_name]);  % Stampa il nome del file per conferma

% Apertura del file per salvare i dati
fileID = fopen(file_name, 'w'); % Apri il file in modalità scrittura
fprintf(fileID, 'angle_steer, desired_filtered_torque, desired_speed_metsec, roll, speed_metsec, torque, u_back_wheel, u_front_wheel, tempo, corrente filtrata, corrente non filtrata, coppia desiderata, volt adc, volt sens, costante D\n'); % Intestazione


% Lettura dati
for i = 1 : N_campioni

    tempo_corrente = (i - 1) * deltaTs; % Calcola il tempo corrente
    while true % Ciclo che continua fino a quando non viene letta una stringa valida
        testo = readline(s); % Legge una linea dalla porta seriale
        testo = strtrim(testo); % Rimuove eventuali spazi bianchi e caratteri di ritorno a capo

        % Controlla se la linea letta contiene il giusto numero di elementi
        dati_letto = textscan(testo, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f"); % Prova a leggere 5 numeri
       
        % Verifica che siano stati letti 5 numeri
        if numel(dati_letto) == 18 && all(~cellfun(@isempty, dati_letto))
            % Estrai solo il primo, terzo e quinto dato
            dati(i, :) = [dati_letto{1}, dati_letto{2},dati_letto{3}, dati_letto{4},dati_letto{5}, dati_letto{6},dati_letto{7}, dati_letto{8},dati_letto{9}, dati_letto{10},dati_letto{11},dati_letto{12},dati_letto{13},dati_letto{14}, dati_letto{15}, dati_letto{16}, dati_letto{17}, dati_letto{18}];
            % Scrivi i dati nel file di testo
            fprintf(fileID, '%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n', dati(i, :));

            
            
           
            % Aggiungi i punti ai grafici animati
            %addpoints(misurato1, tempo_corrente, dati(i, 1)); % Secondo dato
              

            addpoints(misurato2, tempo_corrente, dati(i, 2)); % Terzo dato   addpoints(misurato1, tempo_corrente, dati(i, 1)); % Secondo dato
            %addpoints(misurato3, tempo_corrente, dati(i, 3)); % Terzo dato   addpoints(misurato1, tempo_corrente, dati(i, 1)); % Secondo dato
           % addpoints(misurato6, tempo_corrente, dati(i, 6)); % Terzo dato   addpoints(misurato1, tempo_corrente, dati(i, 1)); % Secondo dato
           % addpoints(misurato4, tempo_corrente, dati(i, 4)/1000); % Terzo dato   addpoints(misurato1, tempo_corrente, dati(i, 1)); % Secondo dato
            addpoints(misurato6, tempo_corrente, dati(i, 6)); % Terzo dato   addpoints(misurato1, tempo_corrente, dati(i, 1)); % Secondo dato
            %addpoints(misurato7, tempo_corrente, dati(i, 7)); % Terzo dato   addpoints(misurato1, tempo_corrente, dati(i, 1)); % Secondo dato
            %addpoints(misurato8, tempo_corrente, dati(i, 8)); % Terzo dato   addpoints(misurato1, tempo_corrente, dati(i, 1)); % Secondo dato
            %addpoints(misurato9, tempo_corrente, dati(i, 9)); % Terzo dato
            %addpoints(misurato10, tempo_corrente, dati(i, 10)); % Secondo dato
            %addpoints(misurato11, tempo_corrente, dati(i, 11)); % Terzo dato   addpoints(misurato1, tempo_corrente, dati(i, 1)); % Secondo dato
            %addpoints(misurato12, tempo_corrente, dati(i, 12)); % Terzo dato   addpoints(misurato1, tempo_corrente, dati(i, 1)); % Secondo dato
           %addpoints(misurato11, tempo_corrente, dati(i, 11)); % Quarto dato
           % addpoints(misurato10, tempo_corrente, dati(i, 10)); % Primo dato
           % addpoints(misurato9, i, dati(i, 9)); % Quarto dato
           % addpoints(misurato8, i, dati(i, 8)); % Primo dato

                                
            drawnow limitrate; % Aggiorna il grafico con una frequenza limitata
            
            break; % Esci dal ciclo `while` solo se la stringa è valida
        else
            fprintf("_stringa non valida o vuota: %s_\n", testo);
            % Se la stringa è vuota o il formato non è corretto, il ciclo `while` continua
        end
    end
end

% Chiusura del file e della porta seriale
fclose(fileID); % Chiudi il file
delete(s); clear s;

