package frc.lib.drivers;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.InetSocketAddress;
import java.net.Socket;

public class NetworkedDatabaseConnection {

    private String hostAddr;
    private int hostPort;
    private final static int TIMEOUT_MS = 100;
    private Socket clientSocket;
    private boolean debug;
    
    public NetworkedDatabaseConnection(){
        this("127.0.0.1", 2222, false);
    }

    public NetworkedDatabaseConnection(String hostAddr, int hostPort, boolean debug){
        this.hostAddr = hostAddr;
        this.hostPort = hostPort;
        this.debug = debug;
    }

    public void reportError(Error e){

    }

    public void reportStatusMessage(MessageType type, String message){

    }

    private void createSocket() {
		try {
			long start = System.currentTimeMillis();
			clientSocket = new Socket();
			clientSocket.connect(new InetSocketAddress(hostAddr, hostPort), TIMEOUT_MS);

			if (debug) {
				long finish = System.currentTimeMillis();
				long timeElapsed = finish - start;
				System.out.println("Socket Creation Time (ms): " + timeElapsed);
			}

        } catch (IOException e) {
            e.printStackTrace();
        }
	}

	private void closeSocket() {
		if (clientSocket != null) {
			try {
				this.clientSocket.close();
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}

    private String sendMessage(String message){
        createSocket();

        String response = null;

		try {
			// Get reader/writer
			PrintWriter out = new PrintWriter(clientSocket.getOutputStream(), true);
            BufferedReader in = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));

			// Send message
			if (debug) {
				System.out.println("SENDING MESSAGE: " + message);
			}
			out.println(message);

			// Read response
			String responseMessage = in.readLine();
			if (debug) {
				System.out.println("GOT RESPONSE: " + responseMessage);
			}

			if (response == null) {
				System.out.println("Unable to parse message.");
			}

		} catch (Exception ex) {
			System.out.println("Could not process message.");

		} finally {
			closeSocket();
		}

        return response;
    }

    public enum MessageType{
        TRACE,
        INFO,
        WARNING,
        ERROR,
        FATAL;
    }

    

}