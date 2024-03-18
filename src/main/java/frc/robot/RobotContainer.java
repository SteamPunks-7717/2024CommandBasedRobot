// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.InputConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.ArrayList;
import java.util.List;

import javax.tools.StandardJavaFileManager.PathFactory;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandJoystick driverJoystick = new CommandJoystick(InputConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // creates named Commands for path planner
    NamedCommands.registerCommand("intakeNote", intakeSubsystem.intakeNote());
    NamedCommands.registerCommand("shootAmp", getShootAmpCommand());
    NamedCommands.registerCommand("shootSpeaker", getShootSpeakerCommand());

    driveSubsystem.setDefaultCommand(
        // The left stick controls translation of the 44robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> driveSubsystem.drive(
                -MathUtil.applyDeadband(driverJoystick.getY() * 0.90, InputConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverJoystick.getX() * 0.90, InputConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverJoystick.getRawAxis(4) * 0.90, InputConstants.kDriveDeadband),
                true, true),
            driveSubsystem));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    new Trigger(driverJoystick.button(1)).onTrue(intakeSubsystem.turnOnMotors());
    new Trigger(driverJoystick.button(1)).onFalse(intakeSubsystem.stopMotors());
    new Trigger(driverJoystick.button(1)).onTrue(shooterSubsystem.shootSpeaker());
    new Trigger(driverJoystick.button(1)).onFalse(shooterSubsystem.stopMotors());

    //climber triggers
    new Trigger(driverJoystick.axisGreaterThan(2, 0.5)).whileTrue(climberSubsystem.retract(1.0));
    new Trigger(driverJoystick.axisGreaterThan(2, 0.5)).onFalse(climberSubsystem.stop());
    new Trigger(driverJoystick.axisGreaterThan(3, 0.5)).whileTrue(climberSubsystem.extend(1.0));
    new Trigger(driverJoystick.axisGreaterThan(3, 0.5)).onFalse(climberSubsystem.stop());

    //servo triggers
    new Trigger(driverJoystick.povLeft()).onTrue(climberSubsystem.servoOut());
    new Trigger(driverJoystick.povRight()).onTrue(climberSubsystem.servoIn());


    new Trigger(driverJoystick.button(6))
        .whileTrue(new RunCommand(
            () -> driveSubsystem.setX(),
            driveSubsystem));

    new Trigger(driverJoystick.button(4)).whileTrue(intakeSubsystem.intakeNote());

    new Trigger(driverJoystick.button(2)).onTrue(
        getShootAmpCommand());

    new Trigger(driverJoystick.button(3)).onTrue(
        getShootSpeakerCommand());

    new Trigger(driverJoystick.button(8)).whileTrue(
       new RunCommand(
        () -> {
          driveSubsystem.zeroHeading();
        }, driveSubsystem)
    );

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {

    return new PathPlannerAuto("Center auto");

    // Pose2d startPoint = new Pose2d(
    //     0.0, 0.0, Rotation2d.fromDegrees(0));

    // Pose2d finalPoint = new Pose2d(
    //     1, 0, Rotation2d.fromDegrees(0));

    // // Interior waypoints here
    // var interiorWaypoints = new ArrayList<Translation2d>();
    // interiorWaypoints.add(new Translation2d(0.30, 0.0));
    // interiorWaypoints.add(new Translation2d(0.60, 0.0));

    // // Create config for trajectory
    // TrajectoryConfig config = new TrajectoryConfig(
    //     AutoConstants.kMaxSpeedMetersPerSecond,
    //     AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //     // Add kinematics to ensure max speed is actually obeyed
    //     .setKinematics(DriveConstants.kDriveKinematics);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     startPoint,
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     interiorWaypoints,
    //     // End 3 meters straight ahead of where we started, facing forward
    //     finalPoint,
    //     config);

    // // Reset odometry to the starting pose of the trajectory.
    // driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return getSwerveControllerCommand(exampleTrajectory).andThen(() -> driveSubsystem.drive(0, 0, 0, false, false))
    //     .andThen(getShootSpeakerCommand()).andThen(shooterSubsystem.stopMotors());
  }

  public SwerveControllerCommand getSwerveControllerCommand(Trajectory _Trajectory) {

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        _Trajectory,
        driveSubsystem::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        driveSubsystem::setModuleStates,
        driveSubsystem);
    return swerveControllerCommand;
  }

  public void doNothing() {

  }

  public SequentialCommandGroup getShootAmpCommand() {
    return new ShootCommand(shooterSubsystem, intakeSubsystem, 0.0, ShooterConstants.AmpLowerMotorRPM, false).andThen(shooterSubsystem.stopMotors());
  }

  public SequentialCommandGroup getShootSpeakerCommand() {
    return new ShootCommand(shooterSubsystem, intakeSubsystem, ShooterConstants.SpeakerRPM, ShooterConstants.SpeakerRPM,
        true).andThen(shooterSubsystem.stopMotors());

  }

}
