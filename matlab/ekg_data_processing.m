%% EKG Projekt WS 2023
% Datum: 24.10.2023

%% Teammitglieder:
% Tamara SUM          73319
% Johannes WERNER     73431

clear 
close all
clc

%% Laden des EGK-Signals

%load("EKG-Daten_tamara3.mat");
load("EKG-Daten_johannes_2.mat");

%% Allgemeine Parameter

N = length(ekg_data);
fA = 250;                           % Abtastfrequenz in Hz
TA = 1/fA;                          % Abtastzeit in s
T = 30;                             % Simulationszeit in Sekunde
t = linspace(0, (N-1) * TA, N);     % Zeitvektor 
% t = linspace(0, T, N);            % Zeitvektor für 30s

FreqAxis = linspace(0, fA, N+1);    % Frequenzachse
FreqAxis = FreqAxis(1:end-1);                   


Fw = fft(ekg_data);                             % Fouriertransformation - Betragspektrum

Fw_A = abs(Fw) / N;                             % Amplitudenspektrum

% Index des Koeffizienten bei 49,5 Hz und 50,5 Hz & (199,5 Hz und 200,5 Hz)
index_49_5Hz = round(49.5 / (fA / N));
index_50_5Hz = round(50.5 / (fA / N));

index_199_5Hz = round(199.5 / (fA / N));
index_200_5Hz = round(200.5  / (fA / N));


% Frequenzbereich zwischen 49,5 Hz und 50,5 Hz (& 199,5 Hz und 200,5 Hz) eliminieren
Fw_filt = Fw;
Fw_filt(index_49_5Hz:index_50_5Hz) = 0;
Fw_filt(index_199_5Hz:index_200_5Hz) = 0;


ekg_data_f = ifft(Fw_filt);                     % Inverse FFT, um das gefilterte Signal zurückzubekommen
Fw_filt_A = abs(Fw_filt) / N;                   % Amplitudenspektrum


%% Plot der EKG-Aufnahme
figure(1)
subplot(2, 1, 1)
plot(t, ekg_data);
xlabel('Zeit in Sekunde');
ylabel('Spannung in Volt');
title('EKG-Signal', 'Rohdaten');
axis([0, 30, 0, 3.3])
grid;

subplot(2, 1, 2)
plot(t, ekg_data_f);
title('', 'Gefiltert');
xlabel('Zeit in Sekunde');
ylabel('Spannung in Volt');
axis([0, 30, 0, 3.3])
grid;


%% Plot des Amplitudenspektrums
figure(2)
subplot(2, 1, 1)
stem(FreqAxis, Fw_A);
title('Amplitudenspektrum des Signals', 'Rohdaten');
xlabel('Frequenz in Hertz)');
ylabel('Amplitude');
grid;

subplot(2, 1, 2)
stem(FreqAxis, Fw_filt_A);
title('','Gefiltert');
xlabel('Frequenz in Hertz');
ylabel('Amplitude');
grid;

%%  Herzratenvariabilität (HRV) 

Rthreshold = 1.7;                   % Schwellwert von R-Zacke
HBcount = 0;                        % Anzahl Herzschläge
lastHB = 0;                         % Zeit des letzten Herzschlags
diffBetweenHB = 0;                  % Vektor mit Zeitdifferenz zw. R-Zacken | RR-Intervall
n = 1;                              % Index äußere while-Schleife
i = 1;                              % Index innere while-Schleife


figure(3)
plot(t, ekg_data);
xlabel('Zeit in Sekunde');
ylabel('Spannung in Volt');
title('EKG-Signal mit R-Zacken-Detektion');
axis([0, 30, 0, 3.3])
hold on
grid;

while (n < length(ekg_data))                   % Go through all data
    if (ekg_data(1,n) >= Rthreshold)           % Look for QRS-Komplexe (R-Zacke)
        while (ekg_data(1,n) >= Rthreshold)    % Look for end of QRS
            n=n+1;
        end
        HBcount = HBcount + 1;                 % Count heartbeat
        diffBetweenHB(i) = t(1,n) - lastHB;    % Save time passed since last heartbeat
        lastHB = t(1,n);
        i=i+1;
        plot(t(1,n),2,'r*');
    end
    n=n+1;
end

bpm = HBcount / 30 * 60;                        % Herzschläge pro Minute
hrv = std(timeSinceLastHB);                     % Herzratenvariabilität
hold off;



