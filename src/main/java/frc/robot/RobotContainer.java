// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.InputConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandJoystick driverJoystick =
      new CommandJoystick(InputConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

  //   driveSubsystem.setDefaultCommand(
  //       // The left stick controls translation of the robot.
  //       // Turning is controlled by the X axis of the right stick.
  //       new RunCommand(
  //           () -> driveSubsystem.drive(
  //               -MathUtil.applyDeadband(driverJoystick.getY(), InputConstants.kDriveDeadband),
  //               -MathUtil.applyDeadband(driverJoystick.getX(), InputConstants.kDriveDeadband),
  //               -MathUtil.applyDeadband(driverJoystick.getRawAxis(4), InputConstants.kDriveDeadband),
  //               false, false),
  //           driveSubsystem));
  }
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
    
    
    new Trigger(driverJoystick.button(1)).onTrue(intakeSubsystem.turnOnMotors());
    new Trigger(driverJoystick.button(1)).onFalse(intakeSubsystem.stopMotors());
    new Trigger(driverJoystick.button(2)).onTrue(shooterSubsystem.shootNote());
    new Trigger(driverJoystick.button(2)).onFalse(shooterSubsystem.stopMotors());

    new Trigger (driverJoystick.button(3))
        .whileTrue(new RunCommand(
            () -> driveSubsystem.setX(),
            driveSubsystem));
    new Trigger (driverJoystick.button(4)).whileTrue(intakeSubsystem.intakeNote());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

  public void doNothing(){
    
  }
}
