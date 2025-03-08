// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.EndEffectorConstants.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class EndEffectorSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private SparkMax fMotor = new SparkMax(fMotorID, MotorType.kBrushless);
  private SparkMax sassyMotor = new SparkMax(sassyMotorID, MotorType.kBrushless);
  private final PIDController sassyMotorPID = new PIDController(sassyMotorkP, sassyMotorkI, sassyMotorkD);

  public EndEffectorSubsystem() {
    
    setDefaultCommand(

        runOnce(

                () -> {
                    
                  sassyMotor.disable();

                })

            .andThen(run(() -> {
            }))

            .withName("Idle"));
  }

  public void spinFMotor(double speed) {
    fMotor.set(speed);
  }
  
  public void spinSassyMotor(double speed) {
    sassyMotor.set(speed);
  }

  public Command moveEndEffectorToSetpoint(double setpointRotationsPerSecond) {
    return parallel(

            run(

                () -> {

                  sassyMotor.set(sassyMotorPID.calculate(sassyMotor.getEncoder().getPosition(), setpointRotationsPerSecond));

                })).withName("Move End Effector to Setpoint");
  }

  public Command endEffectorToL1() {
    return moveEndEffectorToSetpoint(sassyMotorL1SetpointRotationsPerSecond);
  }

  public Command endEffectorToL2() {
    return moveEndEffectorToSetpoint(sassyMotorL2SetpointRotationsPerSecond);
  }

  public Command endEffectorToL3() {
    return moveEndEffectorToSetpoint(sassyMotorL3SetpointRotationsPerSecond);
  }

  public Command endEffectorToL4() {
    return moveEndEffectorToSetpoint(sassyMotorL4SetpointRotationsPerSecond);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("sassyMotor encoder", sassyMotor.getEncoder().getPosition());
  }
}
