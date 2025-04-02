// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static frc.robot.constants.Constants.EndEffectorConstants.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class EndEffectorSubsystem extends SubsystemBase {
  private SparkMax fMotor = new SparkMax(fMotorID, MotorType.kBrushless);
  private SparkMax sassyMotor = new SparkMax(sassyMotorID, MotorType.kBrushless);
  private final PIDController sassyMotorPID = new PIDController(sassyMotorkP, sassyMotorkI, sassyMotorkD);
  private CommandXboxController operatorController = new CommandXboxController(1);

  /** Creates a new EndEffectorSubsystem. */
  public EndEffectorSubsystem() {
    if (operatorController.back().getAsBoolean()) {
      setDefaultCommand(

          runOnce(

              () -> {
                fMotor.disable();
                sassyMotor.disable();
              })

      );
    } else {

      // Set motors to disable when a command is not running
      setDefaultCommand(

          runOnce(

              () -> {
                fMotor.disable();
              }).withTimeout(.1)

              .andThen(
                  holdEndEffector()

              )

      );
    }
  }

  public Command spinFMotor(double speed) {
    return parallel(
        run(

            () -> {

              fMotor.set(speed);

            }

        ));
  }

  public Command spinSassyMotor(double speed) {
    return parallel(
        run(

            () -> {

              sassyMotor.set(speed);

            }

        ));
  }

  public Command moveEndEffectorToSetpoint(double setpoint) {
    return parallel(
        run(

            () -> {
              sassyMotor
                  .set((sassyMotorPID.calculate(sassyMotor.getEncoder().getPosition(), setpoint)) / 2);

            }

        ));

  }

  public boolean getIsEndEffectorAtSetPoint() {
    return sassyMotorPID.atSetpoint();
  }

  public void spinFMotorAtSetpoint(double speed) {
    if (getIsEndEffectorAtSetPoint()) {
      fMotor.set(speed);
    }
  }

  public Command endEffectorUp() {
    return spinSassyMotor(sassyMotorUpSpeed);
  }

  public Command endEffectorDown() {
    return spinSassyMotor(sassyMotorDownSpeed);
  }

  public Command fMotorIn() {
    return spinFMotor(fMotorInSpeed);
  }

  public Command fMotorOut() {
    return spinFMotor(fMotorOutSpeed);
  }

  public Command endEffectorToFeeding() {
    return moveEndEffectorToSetpoint(sassyMotorFeedingSetpoint);
  }

  public Command endEffectorToL1() {
    return moveEndEffectorToSetpoint(sassyMotorL2Setpoint);
  }

  public Command endEffectorToL2() {
    return moveEndEffectorToSetpoint(sassyMotorL2Setpoint);
  }

  public Command endEffectorToL3() {
    return moveEndEffectorToSetpoint(sassyMotorL3Setpoint);
  }

  public Command endEffectorToL4() {
    return moveEndEffectorToSetpoint(sassyMotorL4Setpoint);
  }

  public Command holdEndEffector() {
    return moveEndEffectorToSetpoint(sassyMotorHoldSetpoint);
  }

  public Command endEffectorToRemoveAlgae() {
    return moveEndEffectorToSetpoint(sassyMotorRemoveAlgaeSetpoint);
  }

  public void resetSassyMotorEncoder() {
    sassyMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("sassyMotor encoder", sassyMotor.getEncoder().getPosition());
  }
}