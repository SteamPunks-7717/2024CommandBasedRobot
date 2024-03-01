// Copylower (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANSparkMax motor_shooter_upper;
  private CANSparkMax motor_shooter_lower;
  private double upperSpeed = 0.5;
  private double lowerSpeed = 0.5;

  public ShooterSubsystem() {
    motor_shooter_upper = new CANSparkMax(3,MotorType.kBrushless);
    motor_shooter_lower = new CANSparkMax(4, MotorType.kBrushless);
    SmartDashboard.putNumber("upperSpeed", upperSpeed);
    SmartDashboard.putNumber("lowerSpeed", lowerSpeed);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command shootNote() {
    return runOnce(
        () -> {
        motor_shooter_upper.set(-upperSpeed);
        motor_shooter_lower.set(-lowerSpeed);
        });
  }
  
  public Command stopMotors() {
    return runOnce(
        () -> {
        motor_shooter_upper.set(0);
        motor_shooter_lower.set(0);
        });
  }

  public Command shootSpeaker(){
      return runOnce(
        () -> {
        motor_shooter_upper.set(-ShooterConstants.SpeakerSpeed);
        motor_shooter_lower.set(-ShooterConstants.SpeakerSpeed);
        });
  }

    public Command shootAmp(){
      return runOnce(
        () -> {
        motor_shooter_upper.set(0);
        motor_shooter_lower.set(-ShooterConstants.AmpLowerMotorSpeed);
        });
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    upperSpeed = SmartDashboard.getNumber("upperSpeed", upperSpeed);
    lowerSpeed = SmartDashboard.getNumber("lowerSpeed", lowerSpeed);
    
  } 

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
