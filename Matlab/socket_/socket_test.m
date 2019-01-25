t = tcpip('localhost',50007);

array_test = [1,2,3,4;4,2,5,6;22,3,5,3];
fopen(t);

fwrite(t,'this is test message');
    
bytes = fread(t,[1,t.BytesAvailable]);
float(bytes)