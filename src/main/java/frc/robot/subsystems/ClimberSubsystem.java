// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

  private CANSparkMax climberMotor;
  private Servo climberServo;
  private boolean servoIn;

  /** Creates a new ExampleSubsystem. */
  public ClimberSubsystem() {
    climberMotor = new CANSparkMax(13,MotorType.kBrushed); 
    climberServo = new Servo(0);
    climberServo.set(0.33);
    servoIn = climberServo.getPosition() == 0.33;
    SmartDashboard.putBoolean("ServoIn", servoIn);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command extend(double speed) {
    return runOnce(
        () -> {
          climberMotor.set(speed);
          
        });
  }
  public Command retract(double speed) {
    return runOnce(
        () -> {
          climberMotor.set(-speed);

        });
  }
  public Command stop() {
    return runOnce(
        () -> {
          climberMotor.set(0.0);

        });
  }

  public Command servoIn(){
     return runOnce(
        () -> {
          climberServo.set(0.33);
          servoIn = climberServo.getPosition() == 0.33;
          SmartDashboard.putBoolean("ServoIn", servoIn);
        });
  }

  public Command servoOut(){
     return runOnce(
        () -> {
          climberServo.set(0.0);
          servoIn = climberServo.getPosition() == 0.33;
          SmartDashboard.putBoolean("ServoIn", servoIn);
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
