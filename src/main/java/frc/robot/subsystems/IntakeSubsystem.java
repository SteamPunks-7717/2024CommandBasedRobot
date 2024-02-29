// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANSparkMax motor_intake_upper;
  private CANSparkMax motor_intake_lower;

  private DigitalInput limitSwitch;
  //our limitSwitch returns false when pressed 
  public Boolean switchPressed;

  private double intake_speed = 0.7;

  public IntakeSubsystem() {
    motor_intake_upper = new CANSparkMax(1,MotorType.kBrushless);
    motor_intake_lower = new CANSparkMax(2, MotorType.kBrushless);
    motor_intake_lower.setIdleMode(IdleMode.kBrake);
    motor_intake_upper.setIdleMode(IdleMode.kBrake);
    limitSwitch = new DigitalInput(0);
    switchPressed = !limitSwitch.get();
    SmartDashboard.putBoolean("LimitSwitch",false);
    SmartDashboard.putNumber("intakeSpeed", intake_speed);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command turnOnMotors() {
    return runOnce(
        () -> {
        motor_intake_upper.set(intake_speed);
        motor_intake_lower.set(-intake_speed);
        SmartDashboard.putNumber("intakeSpeed", intake_speed);
        });
  }

  public void spin(){
     motor_intake_upper.set(intake_speed);
      motor_intake_lower.set(-intake_speed);
  }
  public void stop(){
    motor_intake_upper.set(0);
    motor_intake_lower.set(0);
  }
  
  public Command stopMotors() {
    return runOnce(
        () -> {
        motor_intake_upper.set(0);
        motor_intake_lower.set(0);
        SmartDashboard.putNumber("intakeSpeed", intake_speed);
        });
  }

   public Command intakeNote(){
    return new FunctionalCommand(
      () -> switchPressed = !limitSwitch.get(),
      // Action to execute when command is running
      () -> this.spin(),
      // Action to run when command is interrupted
      (interrupted) -> this.stop(),
      // IsFinished (condition to end command)
      () -> this.switchPressed == true,
      // Requirements
      this
  );
  }
  @Override
  public void periodic() {

    switchPressed = !limitSwitch.get();

    intake_speed = SmartDashboard.getNumber("intakeSpeed", intake_speed);
    SmartDashboard.putBoolean("LimitSwitch", switchPressed);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
