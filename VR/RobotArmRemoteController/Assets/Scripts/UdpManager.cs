using System;
using System.Net;
using System.Net.Sockets;
using System.Threading.Tasks;
using UnityEngine;

public class UdpManager
{
    private UdpClient udpClient;
    private IPEndPoint remoteEndPoint;
    private readonly string ipAddress;
    private readonly int port;

    /// <summary>
    /// Constructor that creates a UDP socket for sending messages to the specified IP address and port.
    /// The socket is created once and reused for all subsequent SendMessage calls.
    /// </summary>
    /// <param name="ipAddress">The IP address of the target PC</param>
    /// <param name="port">The port number on the target PC</param>
    public UdpManager(string ipAddress, int port)
    {
        this.ipAddress = ipAddress;
        this.port = port;
        
        try
        {
            // Create the UDP client and remote endpoint
            udpClient = new UdpClient();
            remoteEndPoint = new IPEndPoint(IPAddress.Parse(ipAddress), port);
            
            Debug.Log($"UdpManager initialized. Target: {ipAddress}:{port}");
        }
        catch (Exception ex)
        {
            Debug.LogError($"Failed to initialize UdpManager: {ex.Message}");
        }
    }

    /// <summary>
    /// Sends a UDP message asynchronously to the configured remote endpoint.
    /// This method returns immediately without blocking the calling thread.
    /// </summary>
    /// <param name="messageBytes">The byte array containing the message to send</param>
    public async void SendMessage(byte[] messageBytes)
    {
        if (udpClient == null || remoteEndPoint == null)
        {
            Debug.LogError("UdpManager not properly initialized. Cannot send message.");
            return;
        }

        if (messageBytes == null || messageBytes.Length == 0)
        {
            Debug.LogWarning("Attempted to send empty or null message.");
            return;
        }

        try
        {
            // Send the message asynchronously
            await Task.Run(() => 
            {
                udpClient.Send(messageBytes, messageBytes.Length, remoteEndPoint);
            });
            
            // Optionally log successful sends (comment out in production for performance)
            // Debug.Log($"Sent {messageBytes.Length} bytes to {ipAddress}:{port}");
        }
        catch (Exception ex)
        {
            Debug.LogError($"Failed to send UDP message: {ex.Message}");
        }
    }

    /// <summary>
    /// Cleanup method to properly dispose of the UDP client.
    /// Should be called when the UdpManager is no longer needed.
    /// </summary>
    public void Dispose()
    {
        try
        {
            udpClient?.Close();
            udpClient?.Dispose();
            Debug.Log("UdpManager disposed successfully.");
        }
        catch (Exception ex)
        {
            Debug.LogError($"Error disposing UdpManager: {ex.Message}");
        }
    }

    /// <summary>
    /// Destructor to ensure cleanup if Dispose is not called explicitly.
    /// </summary>
    ~UdpManager()
    {
        Dispose();
    }
}
