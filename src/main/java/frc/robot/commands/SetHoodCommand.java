// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants;
import frc.robot.subsystems.ShootingSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetHoodCommand extends Command {
  /** Creates a new SetHoodCommand. */
  ShootingSubsystem shootingSubsystem;
  PIDController hoodPID;
  CommandPS5Controller operator;
  public SetHoodCommand(ShootingSubsystem shootingSubsystem, CommandPS5Controller operator) {
    this.shootingSubsystem = shootingSubsystem;
    hoodPID = Constants.Shooting.hoodPID;
    this.operator = operator;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shootingSubsystem.hoodAngleMotor.set(-hoodPID.calculate(shootingSubsystem.hoodAngleEncoderValue, 190));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(operator.povUp().getAsBoolean() || operator.povDown().getAsBoolean() || operator.triangle().getAsBoolean()){
      return true;
    }
    return false;
  }
}
