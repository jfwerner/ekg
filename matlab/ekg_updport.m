%% EKG Projekt WS 2023
% Datum: 24.10.2023

%% Teammitglieder:
% Tamara SUM          73319
% Johannes WERNER     73431

clear 
close all
clc

%% Initialisierung
u = udpport("IPV4")                                             % Construct a byte-type udpport object.
ekg_data_raw = [];                                              % Vektor fur raw-EKG Daten
ekg_data_IIR = [];                                              % Vektor fur geilterte EKG Daten
packet_number = 1;

packets = 4;                                                        % Ein Paket entspricht 15 Sekunden Daten.

writeline(u, "Bereit zu empfangen", "192.168.188.151", 420);   % Nachricht an ESP32 (IP-Adresse und den Port)
%writeline(u, "Bereit zu empfangen", "192.168.0.60", 420);

while (packet_number <= packets)                                % Solange Auslesen bis gewollte Paketanzahl erreicht
    
    while (u.NumBytesAvailable == 0)
    disp('I´m just chilling here');
    disp(packet_number);
    end

    disp(u.NumBytesAvailable)
    
    if mod(packet_number, 2) == 0                               % packetnumber ist gerade
        data_packet = read(u, 3750, "uint16");                  % ESP sendet 1 Byte (8bit) Daten
        ekg_data_IIR = [ekg_data_IIR, data_packet];             % Die empfangenen Daten an den EKG-Datenvektor anhängen
    else                                                        % ungerade
        data_packet = read(u, 3750, "uint16");                  % ESP sendet 1 Byte (8bit) Daten
        ekg_data_raw = [ekg_data_raw, data_packet];             % Die empfangenen Daten an den EKG-Datenvektor anhängen
    end

    packet_number = packet_number + 1;

end

% Daten anzeigen                    
% disp(ekg_data);
% disp(data);                                             

t=linspace(0,15*packets/2,length(ekg_data_raw));
ekg_data_raw=ekg_data_raw/4095*3.3;                            % ADC-Werte in Spannung umrechnen
subplot(2,1,1)
plot(t, ekg_data_raw);
xlabel("Zeit (s)");
ylabel("Spannung (V)");
title('Aufgenomme EKG-Daten', 'Rohdaten');
axis([0, 30, 0, 3.3])

subplot(2,1,2)
ekg_data_IIR=ekg_data_IIR/4095*3.3;                            % ADC-Werte in Spannung umrechnen
plot(t, ekg_data_IIR);
xlabel("Zeit (s)");
ylabel("Spannung (V)");
title('', 'Gefilterte Daten mit IIR-Notch-Filter');
axis([0, 30, 0, 3.3])

save("EKG-Daten_raw_johannes_4.mat","ekg_data_raw");           % Die gelesenen Daten werden abgespeichert
save("EKG-Daten_IIR_johannes_4.mat","ekg_data_IIR");           % Die gelesenen Daten werden abgespeichert


flush(u, "output");                                             % Den Ausgabepuffer leeren 
clear u;                                                        % Das udpport-Objekt löschen
