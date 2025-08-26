"""
This file handles UDP communication to and from the PC app.
"""

import asyncio
import socket
from typing import Optional, Callable


class UdpManager:
    def __init__(self, destination_ip: str, port_number: int, on_message_received: Optional[Callable[[bytes], None]] = None):
        """
        Initialize UDP manager for communication to/from the PC app.

        :param destination_ip: IP address to send messages to
        :param port_number: Port number for UDP communication (both sending and receiving)
        :param on_message_received: Callback function called when a message is received
        """
        self.destination_ip = destination_ip
        self.port_number = port_number
        self.on_message_received = on_message_received
        self.transport = None
        self.protocol = None
        self.running = False
        self.socket = None


    async def start(self):
        """Start UDP server to listen for UDP messages"""
        loop = asyncio.get_running_loop()
        
        # Create UDP server
        self.transport, self.protocol = await loop.create_datagram_endpoint(
            lambda: UdpProtocol(self.on_message_received),
            local_addr=('0.0.0.0', self.port_number)
        )

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # This socket is reused for sending messages
        
        self.running = True
        print(f"UDP server listening on port {self.port_number}")


    async def stop(self):
        """Stop UDP server"""
        if self.transport:
            self.transport.close()
            self.running = False
            print("UDP server stopped")
        if self.socket:
            self.socket.close()
            print("UDP socket closed")
    

    async def send_UDP_packet(self, data: bytes):
        """
        Send UDP packet.

        :param data: The data to send
        """        
        # Send UDP packet
        try:
            self.socket.sendto(data, (self.destination_ip, self.port_number))
        except Exception as e:
            print(f"Failed to send UDP packet: {e}")
            if self.socket:
                self.socket.close()
                print("UDP socket closed")



class UdpProtocol(asyncio.DatagramProtocol):
    def __init__(self, on_message_received: Optional[Callable[[bytes], None]] = None):
        self.on_message_received = on_message_received
        
        
    def connection_made(self, transport):
        self.transport = transport
        

    def datagram_received(self, data, addr):
        """ Handle incoming UDP datagram. """
        if self.on_message_received:
            self.on_message_received(data)
    