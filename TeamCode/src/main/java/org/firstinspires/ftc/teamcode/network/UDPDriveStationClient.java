package org.firstinspires.ftc.teamcode.network;

import java.io.IOException;
import java.io.UnsupportedEncodingException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;

public class UDPDriveStationClient {
    private DatagramSocket socket = null;
    private InetAddress address = null;

    private int serverPort;

    public UDPDriveStationClient(String serverIpAddress, int serverPort) {
        this.serverPort = serverPort;
        try {
            address = InetAddress.getByName(serverIpAddress);
            socket = new DatagramSocket();
        } catch (UnknownHostException e) {
            e.printStackTrace();
        } catch (SocketException e) {
            e.printStackTrace();
        }
    }

    public void sendStringUDP(String message) {
        try {
            byte[] byteString = message.getBytes();
            DatagramPacket request = new DatagramPacket(byteString, byteString.length, address, serverPort);
            socket.send(request);
        } catch (UnsupportedEncodingException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
