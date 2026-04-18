// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MusicCommand extends Command {
  Orchestra orchestra;
  IntakeSubsystem intakeSubsystem;
  ShootingSubsystem shootingSubsystem;
  SwerveSubsystem swerveSubsystem;
  String songName;
  /** Creates a new MusicCommand. */
  public MusicCommand(IntakeSubsystem intakeSubsystem, ShootingSubsystem shootingSubsystem, SwerveSubsystem swerveSubsystem, String songName) {
    orchestra = new Orchestra();
    this.intakeSubsystem = intakeSubsystem;
    this.shootingSubsystem = shootingSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    this.songName = songName;
    intakeSubsystem.intakeSpinny.getConfigurator().apply(new AudioConfigs().withAllowMusicDurDisable(true));
    shootingSubsystem.shootWheelsLeft.getConfigurator().apply(new AudioConfigs().withAllowMusicDurDisable(true));
    shootingSubsystem.shootWheelsRight.getConfigurator().apply(new AudioConfigs().withAllowMusicDurDisable(true));
    ((CoreTalonFX) swerveSubsystem.getSwerveDriveConfiguration().modules[0].getDriveMotor().getMotor()).getConfigurator().apply(new AudioConfigs().withAllowMusicDurDisable(true));
    ((CoreTalonFX) swerveSubsystem.getSwerveDriveConfiguration().modules[1].getDriveMotor().getMotor()).getConfigurator().apply(new AudioConfigs().withAllowMusicDurDisable(true));
    ((CoreTalonFX) swerveSubsystem.getSwerveDriveConfiguration().modules[2].getDriveMotor().getMotor()).getConfigurator().apply(new AudioConfigs().withAllowMusicDurDisable(true));
    ((CoreTalonFX) swerveSubsystem.getSwerveDriveConfiguration().modules[3].getDriveMotor().getMotor()).getConfigurator().apply(new AudioConfigs().withAllowMusicDurDisable(true));

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    orchestra.addInstrument(intakeSubsystem.intakeSpinny, 1);
    orchestra.addInstrument(shootingSubsystem.shootWheelsLeft, 2);
    orchestra.addInstrument(shootingSubsystem.shootWheelsRight, 3);
    orchestra.addInstrument((TalonFX)swerveSubsystem.getSwerveDriveConfiguration().modules[0].getDriveMotor().getMotor(), 4);
    orchestra.addInstrument((TalonFX)swerveSubsystem.getSwerveDriveConfiguration().modules[1].getDriveMotor().getMotor(), 5);
    orchestra.addInstrument((TalonFX)swerveSubsystem.getSwerveDriveConfiguration().modules[2].getDriveMotor().getMotor(), 6);
    orchestra.addInstrument((TalonFX)swerveSubsystem.getSwerveDriveConfiguration().modules[3].getDriveMotor().getMotor(), 7);
    var status = orchestra.loadMusic(songName);
    if(status.isOK()) orchestra.play();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    orchestra.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}