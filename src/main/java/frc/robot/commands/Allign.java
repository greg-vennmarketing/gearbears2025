package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Client;
import frc.robot.subsystems.SwerveSubsystem;

public class Allign extends Command {
    Client client;
    SwerveSubsystem drive;
    GoToPoint goToPointCommand;
    GoToPoint goToPointCommand2;

    final double yDifference = 0.1651 + 0.0254;
    final double apriltagCenter = 0.08255;
    boolean left;
    public double difference;

    public int cycle = 0;

    public Allign(Client cli, SwerveSubsystem drive, boolean left) {
        this.drive = drive;
        client = cli;
        this.left = left;
    }

    public void initialize() {
        // get the x and y of the apriltag to move to
        double x, y;
        x = client.position[0];
        y = client.position[1];

        // y = y < 0.1 ? y - 0.025: y + 0.025;

        // if left is true use positive yDifference if not then use negative yDifference
        double difference = left ? yDifference : -yDifference;

        Translation2d movement1 = new Translation2d(x, y - apriltagCenter + difference);
        goToPointCommand = new GoToPoint(drive, movement1, 2);
        goToPointCommand.initialize();
        cycle = 0;
        System.out.println(movement1.getX() + " " + movement1.getY());
    }

    public void execute() {
        goToPointCommand.execute();
    }

    public void end(boolean interupt) {
        goToPointCommand.end(interupt);
        cycle = 0;
    }

    public boolean isFinished() {
        return goToPointCommand.isFinished();
    }
}
