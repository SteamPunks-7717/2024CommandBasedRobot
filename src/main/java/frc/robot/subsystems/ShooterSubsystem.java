// Copylower (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public CANSparkMax motor_shooter_upper;
  public CANSparkMax motor_shooter_lower;

  public SparkPIDController m_shooter_upper_PID;
  public SparkPIDController m_shooter_lower_PID;

  private double upperSpeed = 0.5;
  private double lowerSpeed = 0.5;

  public ShooterSubsystem() {
    motor_shooter_upper = new CANSparkMax(3, MotorType.kBrushless);
    motor_shooter_lower = new CANSparkMax(4, MotorType.kBrushless);

    m_shooter_upper_PID = motor_shooter_upper.getPIDController();
    m_shooter_lower_PID = motor_shooter_lower.getPIDController();


    // set PID coefficients
    m_shooter_upper_PID.setP(ShooterConstants.kP);
    m_shooter_upper_PID.setI(ShooterConstants.kI);
    m_shooter_upper_PID.setD(ShooterConstants.kD);
    m_shooter_upper_PID.setIZone(ShooterConstants.kIz);
    m_shooter_upper_PID.setFF(ShooterConstants.kFF);
    m_shooter_upper_PID.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);

    // set PID coefficients
    m_shooter_lower_PID.setP(ShooterConstants.kP);
    m_shooter_lower_PID.setI(ShooterConstants.kI);
    m_shooter_lower_PID.setD(ShooterConstants.kD);
    m_shooter_lower_PID.setIZone(ShooterConstants.kIz);
    m_shooter_lower_PID.setFF(ShooterConstants.kFF);
    m_shooter_lower_PID.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", ShooterConstants.kP);
    SmartDashboard.putNumber("I Gain", ShooterConstants.kI);
    SmartDashboard.putNumber("D Gain", ShooterConstants.kD);
    SmartDashboard.putNumber("I Zone", ShooterConstants.kIz);
    SmartDashboard.putNumber("Feed Forward", ShooterConstants.kFF);
    SmartDashboard.putNumber("Max Output", ShooterConstants.kMaxOutput);
    SmartDashboard.putNumber("Min Output", ShooterConstants.kMinOutput);

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command shootNote() {
    return runOnce(
        () -> {
          motor_shooter_upper.set(upperSpeed);
          motor_shooter_lower.set(lowerSpeed);
        });
  }

  public Command stopMotors() {
    return runOnce(
        () -> {
          motor_shooter_upper.set(0);
          motor_shooter_lower.set(0);
        });
  }

  public Command shootSpeaker() {
    return runOnce(
        () -> {
          motor_shooter_upper.set(ShooterConstants.SpeakerSpeed);
          motor_shooter_lower.set(ShooterConstants.SpeakerSpeed);

        });
  }

  public Command shootAmp() {
    return runOnce(
        () -> {
          motor_shooter_upper.set(0);
          motor_shooter_lower.set(ShooterConstants.AmpLowerMotorSpeed);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // upperSpeed = SmartDashboard.getNumber("upperSpeed", upperSpeed);
    // lowerSpeed = SmartDashboard.getNumber("lowerSpeed", lowerSpeed);

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    if ((p != ShooterConstants.kP)) {
      m_shooter_lower_PID.setP(p);
      m_shooter_upper_PID.setP(p);
      ShooterConstants.kP = p;
    }
    if ((i != ShooterConstants.kI)) {
      m_shooter_lower_PID.setI(i);
      m_shooter_upper_PID.setI(i);
      ShooterConstants.kI = i;
    }
    if ((d != ShooterConstants.kD)) {
      m_shooter_lower_PID.setD(d);
      m_shooter_upper_PID.setD(i);
      ShooterConstants.kD = d;
    }
    if ((iz != ShooterConstants.kIz)) {
      m_shooter_lower_PID.setIZone(iz);
      m_shooter_upper_PID.setIZone(iz);
      ShooterConstants.kIz = iz;
    }
    if ((ff != ShooterConstants.kFF)) {
      m_shooter_lower_PID.setFF(ff);
      m_shooter_upper_PID.setFF(ff);
      ShooterConstants.kFF = ff;
    }
    if ((max != ShooterConstants.kMaxOutput) || (min != ShooterConstants.kMinOutput)) {
      m_shooter_lower_PID.setOutputRange(min, max);
      m_shooter_upper_PID.setOutputRange(min, max);
      ShooterConstants.kMinOutput = min;
      ShooterConstants.kMaxOutput = max;
    }

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
