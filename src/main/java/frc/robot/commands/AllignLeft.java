package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Client;
import frc.robot.subsystems.SwerveSubsystem;

public class AllignLeft extends Command {
    public double[] position;
    public int id;
    public Client cli;
    public SwerveSubsystem drive;
    int x = 0;
    int y = 1;
    int theta = 2;
    boolean runningCurVel;
    int countX;
    int countY;
    int maxX;
    int maxY;
    double t;

    public AllignLeft(int i, double[] pos, Client c, SwerveSubsystem driveSys) {
        this.id = i;
        this.position = pos;
        this.cli = c;
        this.drive = driveSys;
    }

    public void initialize() {
        id = cli.id;
        position = cli.position;
        runningCurVel = false;
        countX = 0;
        countY = 0;
    }

    public void execute() {
        if (!runningCurVel) {
            id = cli.id;
            position = cli.position;
            position[y] += (position[y] > 0) ? -0.0762 : 0.0762;
            position[x] /= 2;
            maxX = (int)(position[x] * 100);
            maxY = (int)(position[y] * 100) * ((position[y] < 0) ? -1 : 1);
            runningCurVel = true;
        } else {
            if (countX % 10 != 9) {
                double xSpeed = (countX < maxX) ? 0.5 : 0;
                double ySpeed = ((countX < maxY) ? 0.5 : 0) * ((position[y] < 0) ? -1 : 1);
                drive.swerveDrive.drive(new Translation2d(xSpeed, ySpeed), 0, false, false);
                countX++;
                countY++;
            } else {
                id = cli.id;
                position = cli.position;
                position[y] += (position[y] > 0) ? -0.0762 : 0.0762;
                position[x] /= 2;
                maxX = (int)(position[x] * 100);
                maxY = (int)(position[y] * 100) * ((position[y] < 0) ? -1 : 1);
                System.out.println("X: " + position[x] + " Y: " + position[y]);
                countX = 0;
                countY = 0;
            }
        }
    }


    public void end(boolean interupt) {
        System.out.println("it worked: #vision");
        drive.swerveDrive.drive(new Translation2d(0.0, 0.0), 0.0, false, false);
        countX = 0;
        countY = 0;
        runningCurVel = false;
    }

    public boolean isFinished() {
        return countX >= maxX && countY >= maxY;
    }
}
