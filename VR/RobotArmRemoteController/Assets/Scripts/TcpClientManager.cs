using System;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Collections.Concurrent;
using UnityEngine;

public class TcpClientManager : MonoBehaviour
{
    private TcpClient client;
    private NetworkStream stream;
    private Thread receiveThread;
    private bool isRunning;

    // Configurable in Inspector
    public string serverIp = "192.168.0.43";
    public int port = 12345;
    public GameObject targetObject; // Optional: e.g., RobotGripper for data TODO: Get and send data to other script.

    private ConcurrentQueue<string> messageQueue = new ConcurrentQueue<string>();
    private ConcurrentQueue<System.Action> actionQueue = new ConcurrentQueue<System.Action>();

    void Start()
    {
        ConnectToServer();
    }

    void ConnectToServer()
    {
        try
        {
            client = new TcpClient();
            client.Connect(serverIp, port);
            stream = client.GetStream();
            isRunning = true;

            // Start receive thread
            receiveThread = new Thread(ReceiveData);
            receiveThread.IsBackground = true;
            receiveThread.Start();

            // Start sending data at 10 Hz
            InvokeRepeating("SendData", 0f, 0.1f); // 0.1s = 10 Hz
            Debug.Log("Connected to server.");
        }
        catch (Exception e)
        {
            Debug.LogError($"Connection failed: {e.Message}");
        }
    }

    void SendData()
    {
        if (!isRunning || stream == null) return;

        try
        {
            // Example: Send RobotGripper's position as a string
            string data = targetObject != null
                ? $"Position: {targetObject.transform.position.ToString("F2")}"
                : "Hello from Quest 3";
            byte[] buffer = Encoding.UTF8.GetBytes(data + "\n"); // Add newline as delimiter
            stream.Write(buffer, 0, buffer.Length);
            // Debug.Log($"Sent: {data}");
        }
        catch (Exception e)
        {
            Debug.LogError($"Send error: {e.Message}");
            Disconnect();
        }
    }

    void Update()
    {
        // Process received messages on main thread
        while (messageQueue.TryDequeue(out string message))
        {
            HandleReceivedData(message);
        }

        // Process other actions on main thread
        while (actionQueue.TryDequeue(out System.Action action))
        {
            action?.Invoke();
        }
    }

    void ReceiveData()
    {
        byte[] buffer = new byte[1024];
        StringBuilder message = new StringBuilder();

        while (isRunning)
        {
            try
            {
                int bytesRead = stream.Read(buffer, 0, buffer.Length);
                if (bytesRead > 0)
                {
                    string received = Encoding.UTF8.GetString(buffer, 0, bytesRead);
                    message.Append(received);

                    // Process complete messages (assuming newline delimiter)
                    int newlineIndex;
                    while ((newlineIndex = message.ToString().IndexOf('\n')) >= 0)
                    {
                        string completeMessage = message.ToString(0, newlineIndex).Trim();
                        message.Remove(0, newlineIndex + 1);

                        // Queue message for main thread processing
                        messageQueue.Enqueue(completeMessage);
                    }
                }
                else
                {
                    // Server disconnected
                    actionQueue.Enqueue(() =>
                    {
                        Debug.Log("Server disconnected.");
                        Disconnect();
                    });
                    break;
                }
            }
            catch (Exception e)
            {
                actionQueue.Enqueue(() =>
                {
                    Debug.LogError($"Receive error: {e.Message}");
                    Disconnect();
                });
                break;
            }
        }
    }

    void HandleReceivedData(string data)
    {
        // Example: Log received data or update GameObject
        Debug.Log($"Received: {data}");

        // Example action: If data is a position, move targetObject
        if (targetObject != null && data.StartsWith("MoveTo:"))
        {
            string[] parts = data.Substring(7).Split(',');
            if (parts.Length == 3 &&
                float.TryParse(parts[0], out float x) &&
                float.TryParse(parts[1], out float y) &&
                float.TryParse(parts[2], out float z))
            {
                targetObject.transform.position = new Vector3(x, y, z);
            }
        }
    }

    void Disconnect()
    {
        isRunning = false;
        stream?.Close();
        client?.Close();
        receiveThread?.Join();
        stream = null;
        client = null;
        CancelInvoke("SendData");
        Debug.Log("Disconnected from server.");
    }

    void OnDestroy()
    {
        Disconnect();
    }
}