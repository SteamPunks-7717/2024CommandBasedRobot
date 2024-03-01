package frc.robot.commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private double upper_setPoint;
  private double lower_setPoint;
  private double counter = 0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootCommand(ShooterSubsystem subsystem, double _upper_setPoint, double _lower_setPoint) {
    shooterSubsystem = subsystem;
    upper_setPoint = _upper_setPoint;
    lower_setPoint = _lower_setPoint;
    counter = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double upper_normaliazed_set_point = upper_setPoint * ShooterConstants.maxRPM;
    double lower_normaliazed_set_point = lower_setPoint * ShooterConstants.maxRPM;

    shooterSubsystem.m_shooter_upper_PID.setReference(upper_normaliazed_set_point, CANSparkMax.ControlType.kVelocity);
    shooterSubsystem.m_shooter_lower_PID.setReference(lower_normaliazed_set_point, CANSparkMax.ControlType.kVelocity);

    SmartDashboard.putNumber("SetPoint Upper", upper_normaliazed_set_point);
    SmartDashboard.putNumber("SetPoint Lower", lower_normaliazed_set_point);

    SmartDashboard.putNumber("Upper variable (RPM)", shooterSubsystem.motor_shooter_upper.getEncoder().getVelocity());
    SmartDashboard.putNumber("Lower variable (RPM)", shooterSubsystem.motor_shooter_lower.getEncoder().getVelocity());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    counter++;
    SmartDashboard.putNumber("Upper variable (RPM)", shooterSubsystem.motor_shooter_upper.getEncoder().getVelocity());
    SmartDashboard.putNumber("Lower variable (RPM)", shooterSubsystem.motor_shooter_lower.getEncoder().getVelocity());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    double upper_normaliazed_set_point = upper_setPoint * ShooterConstants.maxRPM;
    double lower_normaliazed_set_point = lower_setPoint * ShooterConstants.maxRPM;

    double upper_pid_error = Math
        .abs(shooterSubsystem.motor_shooter_lower.getEncoder().getVelocity() - lower_normaliazed_set_point);
    double lower_pid_error = Math
        .abs(shooterSubsystem.motor_shooter_upper.getEncoder().getVelocity() - upper_normaliazed_set_point);

    if (upper_pid_error <= ShooterConstants.PID_acceptable_threshold
        && lower_pid_error <= ShooterConstants.PID_acceptable_threshold) {
      if (counter >= ShooterConstants.counter_max) {
        counter = 0;
        return true;

      }else{
        
      }

    }

    return false;
  }
}
