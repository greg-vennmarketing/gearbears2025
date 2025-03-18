package frc.robot;


import java.io.BufferedInputStream;
import java.io.DataInputStream;
import java.io.IOException;
import java.net.Socket;


public class Client {
   Socket sock;
   int connection;
   DataInputStream in;

   public int id;
   public double[] position = new double[3];

   public Client(int port) {
       this.connection = port;
       try {
           this.sock = new Socket("10.94.80.13", port);
           in = new DataInputStream(new BufferedInputStream(sock.getInputStream()));
       } catch (IOException e) {
           e.printStackTrace();
       }
   }

   public void read() throws IOException {

       try {
            String out = "";
            byte b = in.readByte();
            while ((char)b != '\n') {
                out += (char) b;
                b = in.readByte();
            }
            String[] nums = out.strip().split(",");
            if (Integer.parseInt(nums[0]) != 0){
                id = Integer.parseInt(nums[0]);
                for (int i = 0; i < 3; i++) {
                    // System.out.println("expected " + nums[i + 1]);
                    position[i] = (double) Integer.parseInt(nums[i + 1]) / 1000;
                    // System.out.println("seen " + position[i]);
                }
                printVal();
            }
            // printNums(nums);
            out = "";
        } catch (java.lang.NumberFormatException e) {
            System.out.println("closing");
            close();
        }
        
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
