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
load("EKG-Daten_raw_johannes_4.mat");
load("EKG-Daten_IIR_johannes_4.mat");
%load("EKG-Daten_johannes_2.mat");

%ekg_data_raw = ekg_data;

%% Allgemeine Parameter

N = length(ekg_data_raw);
fA = 250;                           % Abtastfrequenz in Hz
TA = 1/fA;                          % Abtastzeit in s
%T = 30;                             % Simulationszeit in Sekunde
t = linspace(0, (N-1) * TA, N);     % Zeitvektor 

FreqAxis = linspace(0, fA, N+1);    % Frequenzachse
FreqAxis = FreqAxis(1:end-1);                   


Fw_raw = fft(ekg_data_raw);                         % Fouriertransformation - Betragspektrum
Fw_IIR = fft(ekg_data_IIR);
Fw_raw_A = abs(Fw_raw) / N;                             % Amplitudenspektrum
Fw_IIR_A = abs(Fw_IIR) / N;


% Index des Koeffizienten bei 49,5 Hz und 50,5 Hz & (199,5 Hz und 200,5 Hz)
index_49_5Hz = round(49.5 / (fA / N));
index_50_5Hz = round(50.5 / (fA / N));

index_199_5Hz = round(199.5 / (fA / N));
index_200_5Hz = round(200.5  / (fA / N));


% Frequenzbereich zwischen 49,5 Hz und 50,5 Hz (& 199,5 Hz und 200,5 Hz) eliminieren
Fw_filt = Fw_raw;
Fw_filt(index_49_5Hz:index_50_5Hz) = 0;
Fw_filt(index_199_5Hz:index_200_5Hz) = 0;


ekg_data_filt = ifft(Fw_filt);                      % Inverse FFT, um das gefilterte Signal zurückzubekommen
Fw_filt_A = abs(Fw_filt) / N;                       % Amplitudenspektrum


%% Plot der EKG-Aufnahme
figure(1)
subplot(3, 1, 1)
plot(t, ekg_data_raw);
xlabel('Zeit in Sekunde');
ylabel('Spannung in Volt');
title('EKG-Signal', 'Rohdaten');
axis([0, 30, 0, 3.3])
grid;

subplot(3, 1, 2)
plot(t, ekg_data_filt);
title('', 'Gefiltert in Matlab');
xlabel('Zeit in Sekunde');
ylabel('Spannung in Volt');
axis([0, 30, 0, 3.3])
grid;

subplot(3, 1, 3)
plot(t, ekg_data_IIR);
title('', 'Gefiltert mit digitalem IIR-Notch-Filter');
xlabel('Zeit in Sekunde');
ylabel('Spannung in Volt');
axis([0, 30, 0, 3.3])
grid;

%% Plot des Amplitudenspektrums
figure(2)
subplot(3, 1, 1)
stem(FreqAxis, Fw_raw_A);
title('Amplitudenspektrum des Signals', 'Rohdaten');
xlabel('Frequenz in Hertz');
ylabel('Amplitude');
xlim([0, 55]);
grid;

subplot(3, 1, 2)
stem(FreqAxis, Fw_filt_A);
title('','Gefiltert in Matlab');
xlabel('Frequenz in Hertz');
ylabel('Amplitude');
xlim([0, 55]);
grid;

subplot(3, 1, 3)
stem(FreqAxis, Fw_IIR_A);
title('','Gefiltert mit digitalem IIR-Notch-Filter');
xlabel('Frequenz in Hertz');
ylabel('Amplitude');
xlim([0, 55]);
grid;

%%  Herzratenvariabilität (HRV) 

Rthreshold = 1.7;                   % Schwellwert von R-Zacke
HBcount = 0;                        % Anzahl Herzschläge
lastHB = 0;                         % Zeit des letzten Herzschlags
diffBetweenHB = 0;                  % Vektor mit Zeitdifferenz zw. R-Zacken | RR-Intervall
n = 1;                              % Index äußere while-Schleife
i = 1;                              % Index innere while-Schleife

figure(3)
% subplot (2, 1, 1)
plot(t, ekg_data_raw);
xlabel('Zeit in Sekunde');
ylabel('Spannung in Volt');
title('EKG-Signal mit R-Zacken-Detektion');
axis([0, 30, 0, 3.3])
hold on
grid;

while (n < length(ekg_data_raw))                   % Go through all data
    if (ekg_data_raw(1,n) >= Rthreshold)           % Look for QRS-Komplexe (R-Zacke)
        while (ekg_data_raw(1,n) >= Rthreshold)    % Look for end of QRS
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

% EKG to C++ array
% ekg_data_raw=ekg_data_raw/3.3*4095
% fmt=['uint%d_t %s={' repmat('%d,',1,numel(ekg_data_raw)-1) '%d}'];
% c_code=sprintf(fmt,8,'ekg_data',ekg_data_raw)


