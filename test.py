import socket

UDP_IP = "127.0.0.1"  # Replace with the target IP address
UDP_PORT = 5005       # Replace with the target port

message = "Hello, UDP!"  # send the message via socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

try:
    sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
    print(f"Sent UDP packet to {UDP_IP}:{UDP_PORT}: {message}")
finally:
    sock.close()