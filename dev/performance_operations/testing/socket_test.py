import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
result = sock.connect_ex(('192.168.1.10', 30004))
print("Port 30004 reachable:", result == 0)
sock.close()