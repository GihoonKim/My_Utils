import socket
import struct

HOST = ''
PORT = 50007

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(1)

conn, addr = s.accept()

print('Connected by',addr)

while 1:
    data = conn.recv(1024)
    data_ = struct.unpack('>d',data)
    print(data_)
    if not data: break

    # msg1 = struct.pack('>d',data+3.0)

    conn.send(data)

print('done')
conn.close
