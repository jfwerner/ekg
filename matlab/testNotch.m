w0 = 50/250;    %Annahme 200Hz Abtastfrequenz
bw = w0/35;
[num,den] = iirnotch(w0,bw);