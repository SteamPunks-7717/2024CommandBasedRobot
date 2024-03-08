// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

  private CANSparkMax climberMotor;

  /** Creates a new ExampleSubsystem. */
  public ClimberSubsystem() {
    climberMotor = new CANSparkMax(13,MotorType.kBrushed); 
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command extend() {
    return runOnce(
        () -> {
          climberMotor.set(0.2);

        });
  }
  public Command retract() {
    return runOnce(
        () -> {
          climberMotor.set(-0.2);

        });
  }
  public Command stop() {
    return runOnce(
        () -> {
          climberMotor.set(0.0);

        });
  }
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
