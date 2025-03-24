// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static frc.robot.constants.Constants.EndEffectorConstants.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class EndEffectorSubsystem extends SubsystemBase {
  private SparkMax fMotor = new SparkMax(fMotorID, MotorType.kBrushless);
  private SparkMax sassyMotor = new SparkMax(sassyMotorID, MotorType.kBrushless);
  private final PIDController sassyMotorPID = new PIDController(sassyMotorkP, sassyMotorkI, sassyMotorkD);

  /** Creates a new EndEffectorSubsystem. */
  public EndEffectorSubsystem() {

    // Set motors to disable when a command is not running
    setDefaultCommand(

        runOnce(

            () -> {

              sassyMotor.disable();
              fMotor.disable();

            })

            .andThen(run(() -> {
            }))

            .withName("Idle"));
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

  public Command moveEndEffectorToSetpoint(double setpointRotationsPerSecond) {
    return parallel(
        run(

            () -> {

              sassyMotor
                  .set(sassyMotorPID.calculate(sassyMotor.getEncoder().getPosition(), setpointRotationsPerSecond));

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

  public Command EndEffectorUp() {
    return spinSassyMotor(sassyMotorUpSpeed);
  }

  public Command EndEffectorDown() {
    return spinSassyMotor(sassyMotorDownSpeed);
  }

  public Command FMotorIn() {
    return spinFMotor(fMotorInSpeed);
  }

  public Command FMotorOut() {
    return spinFMotor(fMotorOutSpeed);
  }

  public Command EndEffectorToFeeding() {
    return moveEndEffectorToSetpoint(sassyMotorFeedingSetpointRotationsPerSecond);
  }

  public Command EndEffectorToL1() {
    return moveEndEffectorToSetpoint(sassyMotorL2SetpointRotationsPerSecond);
  }

  public Command EndEffectorToL2() {
    return moveEndEffectorToSetpoint(sassyMotorL2SetpointRotationsPerSecond);
  }

  public Command EndEffectorToL3() {
    return moveEndEffectorToSetpoint(sassyMotorL3SetpointRotationsPerSecond);
  }

  public Command HoldEndEffector() {
    return moveEndEffectorToSetpoint(sassyMotorHoldSetpointRotationsPerSecond);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("sassyMotor encoder", sassyMotor.getEncoder().getPosition());
  }
}