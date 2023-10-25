% EKG Projekt WS 2023
% Datum: 24.10.2023

% Teammitglieder:
% Tamara SUM          73319
% Johannes WERNER     73431


u = udpport("IPV4")                                     % Construct a byte-type udpport object.

%write(u,1:5,"uint8","192.168.188.151", 420);           % Write a vector of uint8 data via the udpport socket to a specified address and port.

configureTerminator(u,"CR/LF");
writeline(u,"Halloooooooooo","192.168.188.151",420);

%data = readline(u)
data = read(u,10,"uint8");                             % Read 10 values of uint16 data from the udpport socket.
fprintf('Number of bytes received: %f', data);

flush(u,"output");
clear u
