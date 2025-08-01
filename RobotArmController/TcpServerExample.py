import socket
import threading
import sys

def main():
    server_ip = "0.0.0.0"  # Listen on all interfaces
    server_port = 12345     # Same port as in Unity script

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        server.bind((server_ip, server_port))
        server.listen(5)
        print(f"Server started on port {server_port}...")
        
        while True:
            client, address = server.accept()
            print(f"Client connected: {address}")
            client_thread = threading.Thread(target=handle_client, args=(client, address))
            client_thread.start()
    except Exception as e:
        print(f"Server error: {e}")
    finally:
        server.close()

def handle_client(client_socket, address):
    buffer_size = 1024
    message = ""

    try:
        while True:
            data = client_socket.recv(buffer_size).decode('utf-8')
            if not data:
                break

            message += data
            while '\n' in message:
                complete_message, message = message.split('\n', 1)
                complete_message = complete_message.strip()
                if complete_message:
                    print(f"Received from {address}: {complete_message}")

                    # Example response: Send a position
                    response = "MoveTo:1.0,2.0,3.0\n"
                    client_socket.send(response.encode('utf-8'))
    except Exception as e:
        print(f"Client {address} error: {e}")
    finally:
        client_socket.close()
        print(f"Client {address} disconnected.")

if __name__ == "__main__":
    main()