// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static frc.robot.constants.Constants.ClimberConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private static SparkMax climberMotor = new SparkMax(climberMotorID, MotorType.kBrushless);
  private EndEffectorSubsystem endEffector;
  private LEDSubsystem ledSubsystem;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem(EndEffectorSubsystem m_endEffector, LEDSubsystem m_ledSubsystem) {

    endEffector = m_endEffector;
    ledSubsystem = m_ledSubsystem;

    setDefaultCommand(

        runOnce(

            () -> {

              climberMotor.disable();

            })

            .andThen(run(() -> {
            }))

            .withName("Idle"));
  }

  public Command windClimber() {
    return parallel(

        run(() -> {

          climberMotor.set(climberMotorSpeed);

        }),

        endEffector.holdEndEffector(),
        ledSubsystem.setLED(Constants.LEDConstants.scrollingRainbow));
  }

  public Command unwindClimber() {
    return parallel(

        run(() -> {

          climberMotor.set(-climberMotorSpeed);

        }),

        endEffector.holdEndEffector(), 
        ledSubsystem.setLED(Constants.LEDConstants.scrollingRainbowSlow));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
