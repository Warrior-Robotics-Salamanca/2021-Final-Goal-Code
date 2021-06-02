package org.firstinspires.ftc.teamcode.network;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.net.*;
import java.io.*;
import java.util.ArrayList;

public class TCPDriveStationClient {
    private Telemetry telemetry;
    private ArrayList<String> sendBuffer = new ArrayList<String>();
    private ArrayList<String> receiveBuffer = new ArrayList<String>();

    private Socket socket;
    private OutputStream output;
    private PrintWriter writer;

    public String hostname;
    public int port;

    //private boolean connectedToDriverStation;

    public TCPDriveStationClient(Telemetry telemetry, String hostname, int port) {
        this.telemetry = telemetry;
        this.hostname = hostname;
        this.port = port;

        telemetry.addData("Robot Network Client:", "Client starting on port " + port + " and connecting to " + hostname);
        telemetry.update();

        connectToDriverStation(hostname, port);
    }

    private void connectToDriverStation(String hostname, int port) {
        if(socket == null || !socket.isConnected()) {
            try {
                socket = new Socket(hostname, port);

                output = socket.getOutputStream();
                writer = new PrintWriter(output, true);
            } catch (UnknownHostException ex) {
                telemetry.addData("Server not found: ", ex.getMessage());
                telemetry.update();
            } catch (IOException ex) {
                socket = null;
                telemetry.addData("I/O error: ", ex.getMessage());
                telemetry.update();
            }
        }
    }

    public String exchangeData(String data) {
        connectToDriverStation(hostname, port);
        String response = "";
        try {
            writer.println(data);

            InputStream input = socket.getInputStream();
            BufferedReader reader = new BufferedReader(new InputStreamReader(input));

            response = reader.readLine();
        } catch (IOException ex) {
            socket = null;
            telemetry.addData("I/O error: ", ex.getMessage());
            telemetry.update();
        }
        if(response.equals("ACK"))
            response = "";
        return response;
    }

    public void shutdownClient() {
        try {
            socket.close();
        } catch (IOException ex) {
            telemetry.addData("I/O error: " , ex.getMessage());
            telemetry.update();
        }
    }
}
