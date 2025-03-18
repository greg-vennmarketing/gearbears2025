// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AllignLeft;
import frc.robot.commands.Autos;
import frc.robot.commands.HomeElevator;
import frc.robot.commands.L1Command;
import frc.robot.commands.L4Command;
import frc.robot.commands.Shoot;
import frc.robot.commands.StopShoot;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EffectorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public Client cli;
  // The robot's subsystems and commands are defined here...
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final EffectorSubsystem effector = new EffectorSubsystem();
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final XboxController m_mechanismController =
      new XboxController(OperatorConstants.kMechanismControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    cli = new Client(10005);
    // Configure the trigger bindings
    DriverStation.silenceJoystickConnectionWarning(true);
    configureBindings();
    drivebase.setDefaultCommand(!RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);

//pathplanner commands
    NamedCommands.registerCommand("Shoot", new Shoot(effector).withTimeout(1));
    NamedCommands.registerCommand("StopShoot", new StopShoot(effector).withTimeout(0.1));
    NamedCommands.registerCommand("L1Command", new L1Command(elevator));
    NamedCommands.registerCommand("L4Command", new L4Command(elevator));
    NamedCommands.registerCommand("HomeElevator", new HomeElevator(elevator));

  }

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                                            () -> m_driverController.getLeftY() * -1,
                                                                                            () -> m_driverController.getLeftX() * -1)
                                                                                          .withControllerRotationAxis(m_driverController::getRightX)
                                                                                          .deadband(OperatorConstants.DEADBAND)
                                                                                          .scaleTranslation(0.8)
                                                                                          .allianceRelativeControl(true);
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX,
                                                                                            m_driverController::getRightY)
                                                        .headingWhile(true);




  Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);



  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -m_driverController.getLeftY(),
                                                                        () -> -m_driverController.getLeftX())
                                                                        .withControllerRotationAxis(() -> m_driverController.getRawAxis(2))
                                                                        .deadband(OperatorConstants.DEADBAND)
                                                                        .scaleTranslation(0.8)
                                                                        .allianceRelativeControl(true);
// Derive the heading axis with math!
SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
         .withControllerHeadingAxis(() ->
                                        Math.sin(
                                            m_driverController.getRawAxis(
                                                2) *
                                            Math.PI) *
                                        (Math.PI *
                                         2),
                                    () ->
                                        Math.cos(
                                            m_driverController.getRawAxis(
                                                2) *
                                            Math.PI) *
                                        (Math.PI *
                                         2))
         .headingWhile(true);

Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleKeyboard);

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

   private void configureBindings() {
    new JoystickButton(m_mechanismController, XboxController.Button.kA.value)
        .onTrue(new InstantCommand(() -> elevator.setPositionInches(ElevatorConstants.L1)));

    new JoystickButton(m_mechanismController, XboxController.Button.kB.value)
        .onTrue(new InstantCommand(() -> elevator.setPositionInches(ElevatorConstants.L2)));

        
    new JoystickButton(m_mechanismController, XboxController.Button.kX.value)
        .onTrue(new InstantCommand(() -> elevator.setPositionInches(ElevatorConstants.L3)));

    new JoystickButton(m_mechanismController, XboxController.Button.kY.value)
        .onTrue(new InstantCommand(() -> elevator.setPositionInches(ElevatorConstants.L4)));

    new JoystickButton(m_mechanismController, XboxController.Button.kBack.value)
        .onTrue(new InstantCommand(() -> elevator.setPositionInches(ElevatorConstants.downPos)));

    m_driverController.leftBumper().onTrue(new AllignLeft(cli.id, cli.position, cli, drivebase));

new JoystickButton(m_mechanismController, XboxController.Button.kRightBumper.value)
    .whileTrue(new InstantCommand(() -> effector.shoot()))
    .onFalse(new InstantCommand(() -> effector.stop()));


new JoystickButton(m_mechanismController, XboxController.Button.kLeftBumper.value)
    .whileTrue(new InstantCommand(() -> effector.intake()))
    .onFalse(new InstantCommand(() -> effector.stop()));






    






  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("TLFOUR");
  }


}
