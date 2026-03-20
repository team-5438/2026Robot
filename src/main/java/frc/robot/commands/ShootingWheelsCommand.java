// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.Robot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootingWheelsCommand extends Command {
  ShootingSubsystem shootingSubsystem;
  SpencerAutoAim spencerAutoAim;
  double speed;
  
  /** Creates a new ShootCommand. */
  public ShootingWheelsCommand(ShootingSubsystem shootingSubsystem, SpencerAutoAim spencerAutoAim, double speed) {
    this.shootingSubsystem = shootingSubsystem;
    this.spencerAutoAim = spencerAutoAim;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shootingSubsystem.shootWheelsRight.set(speed); //NOT TESTED SPEED
    shootingSubsystem.shootWheelsLeft.set(-speed); //NOT TESTED SPEED
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(spencerAutoAim.autoAimOn){
      shootingSubsystem.shootWheelsRight.set(0.1); //NOT TESTED SPEED
      shootingSubsystem.shootWheelsLeft.set(-0.1); //NOT TESTED SPEED
    } else {
      shootingSubsystem.shootWheelsRight.set(0);
      shootingSubsystem.shootWheelsLeft.set(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
