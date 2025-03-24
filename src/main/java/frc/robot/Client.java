package frc.robot;


import java.io.BufferedInputStream;
import java.io.DataInputStream;
import java.io.IOException;
import java.io.PrintWriter;
import java.net.Socket;


public class Client {
   Socket sock;
   int connection;
   DataInputStream in;
   PrintWriter o;

   public int id;
   public double[] position = new double[3];
   
   String out = "";

   int counter = 0;
   public Client(int port) {
       this.connection = port;
       try {
           this.sock = new Socket("10.94.80.13", port);
           in = new DataInputStream(new BufferedInputStream(sock.getInputStream()));
           o = new PrintWriter(sock.getOutputStream());
    } catch (IOException e) {
           System.out.println("yeah didn't work");
       }
   }

   public void askForIn() {
    if (this.o != null) {
        o.println("PLS");
        o.flush();
        counter++;
    } else {
        System.out.println(sock);
    }
   }

   public void read() throws IOException {
        // System.out.println("got here");
        try {
            if (in != null) {
                byte b = in.readByte();
                while ((char) b != '\n') {
                    out += (char) b;
                    b = in.readByte();
                }
                if ((char) b == '\n') {
                    String[] nums = out.strip().split(",");
                    id = Integer.parseInt(nums[0]);
                    for (int i = 0; i < 3; i++) {
                        position[i] = (double) Integer.parseInt(nums[i + 1]) / 1000;
                    }
                    out = "";
                }
            } else {
                id = -1;
                position[0] = -1.0; position[1] = -1.0; position[2] = -1.0;
                System.out.println("-1,-1,-1,-1");
            }
        } catch (java.lang.NumberFormatException e) {
            System.out.println("closing");
            close();
        }
        counter++;
    }

    public void printVal() {
        System.out.print("id: " + id + " val: ");
        for (int i = 0; i < position.length; i++) {
            System.out.print(position[i] + " ");
        }
        System.out.println(" ");
    }
    public void printNums(String[] nums) {
        for (int i = 0; i < nums.length; i++) {
            System.out.print(nums[i] + " ");
        }
        System.out.println(" ");
        System.out.println(nums.length);
    }

   public void close() throws IOException {
       sock.close();
       in.close();
   }
}