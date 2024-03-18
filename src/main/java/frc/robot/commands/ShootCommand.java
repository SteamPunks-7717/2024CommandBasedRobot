package frc.robot.commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private double upper_setPoint;
  private double lower_setPoint;
  private double counter = 0;
  private double end_counter = 0;
  private boolean noteShot = false;
  private boolean shootingSpeaker = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootCommand(ShooterSubsystem subsystem, IntakeSubsystem subsystem2, double _upper_setPoint,
      double _lower_setPoint, boolean _shootingSpeaker) {
    shooterSubsystem = subsystem;
    intakeSubsystem = subsystem2;
    upper_setPoint = _upper_setPoint;
    lower_setPoint = _lower_setPoint;
    shootingSpeaker = _shootingSpeaker;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  if (shootingSpeaker){
    shooterSubsystem.m_shooter_upper_PID.setReference(upper_setPoint, CANSparkMax.ControlType.kVelocity);
  }
    shooterSubsystem.m_shooter_lower_PID.setReference(lower_setPoint, CANSparkMax.ControlType.kVelocity);

  //  SmartDashboard.putNumber("SetPoint Upper", upper_setPoint);
  //  SmartDashboard.putNumber("SetPoint Lower", lower_setPoint);


    end_counter = 0;
    counter = 0;
    noteShot = false;

    SmartDashboard.putNumber("Upper variable (RPM)", shooterSubsystem.motor_shooter_upper.getEncoder().getVelocity());
    SmartDashboard.putNumber("Lower variable (RPM)", shooterSubsystem.motor_shooter_lower.getEncoder().getVelocity());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Upper variable (RPM)", shooterSubsystem.motor_shooter_upper.getEncoder().getVelocity());
    SmartDashboard.putNumber("Lower variable (RPM)", shooterSubsystem.motor_shooter_lower.getEncoder().getVelocity());
    //lower_setPoint = SmartDashboard.getNumber("SetPoint Lower", lower_setPoint);

    double upper_pid_error = Math
        .abs(shooterSubsystem.motor_shooter_lower.getEncoder().getVelocity() - lower_setPoint);
    double lower_pid_error = Math
        .abs(shooterSubsystem.motor_shooter_upper.getEncoder().getVelocity() - upper_setPoint);

    if (upper_pid_error <= ShooterConstants.PID_acceptable_threshold && lower_pid_error <= ShooterConstants.PID_acceptable_threshold) {
      counter++;
      if (counter >= ShooterConstants.counter_max) {
        intakeSubsystem.spin(0.7);
        noteShot = true;

      }
    }else{
      counter = 0;
    }

    if (noteShot) {
      end_counter++;
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    intakeSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (end_counter>(1.25/0.02)){
      return true;
    }
    return false;
  }
}
