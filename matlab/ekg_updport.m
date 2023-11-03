%% EKG Projekt WS 2023
% Datum: 24.10.2023

%% Teammitglieder:
% Tamara SUM          73319
% Johannes WERNER     73431

clear 
close all

%% Initialisierung
u = udpport("IPV4")                                             % Construct a byte-type udpport object.
ekg_data = [];                                                  % Vektor fur EKG Daten
packet_number = 1;

% Aufnahmezeit. Ein packet entspricht 15 Sekunden Daten.
packets = 3;

writeline(u, "Bereit zu empfangen", "192.168.188.151", 420);    % Nachricht an ESP32 (IP-Adresse und den Port)
%writeline(u, "Bereit zu empfangen", "192.168.0.60", 420);


while (packet_number <= packets)                                       % Solange Auslesen bis mindestens 7500 Werte gelesen wurden (30s)
    
    while (u.NumBytesAvailable == 0)
    disp('Hallo ich warte du Hs');
    disp(packet_number);
    end

    %packet_size = u.NumBytesAvailable/2;
    disp(u.NumBytesAvailable)

    data_packet = read(u, 3750, "uint16");                      % ESP sendet 1 Byte (8bit) Daten
    
    ekg_data = [ekg_data, data_packet];                     % Die empfangenen Daten an den EKG-Datenvektor anhängen
   
    %end

    packet_number = packet_number + 1;

end


% Daten anzeigen                    
%disp(ekg_data);
%disp(data);                                             

t=linspace(0,15*packet_number,length(ekg_data));
ekg_data=ekg_data/4095*3.3; % In Spannung umrechnen
plot(t,ekg_data);
xlabel("Zeit (s)");
ylabel("Spannung (V)");
title("EKG Daten ESP32");
%save("EKG-Daten.mat","ekg_data");                               % Die gelesenen Daten werden abgespeichert

flush(u, "output");                                             % Den Ausgabepuffer leeren 
clear u;                                                        % Das udpport-Objekt löschen
