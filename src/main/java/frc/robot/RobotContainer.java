// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.SetIntakeCommand;
import frc.robot.commands.FeedCommand;
import frc.robot.commands.IntakeWheelsCommand;
import frc.robot.commands.ShootingWheelsCommand;
import frc.robot.commands.SpencerAutoAim;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import gg.questnav.questnav.QuestNav;
import swervelib.SwerveDrive;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public Vision vision;
  public SwerveDrive swerveDrive;
  public SwerveSubsystem swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  public ShootingSubsystem shootingSubsystem = new ShootingSubsystem();
  public QuestNavSubsystem questNavSubsystem = new QuestNavSubsystem(swerveSubsystem);
  public IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public final CommandXboxController driver = new CommandXboxController(Constants.Driver.id);
  public final CommandPS5Controller operator = new CommandPS5Controller(Constants.Operator.id);
  public SpencerAutoAim spencerAutoAim;

  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    
    //Create the NamedCommands that will be used in PathPlanner
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    //Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();

    //Set the default auto (do nothing) 
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    //Add a simple auto option to have the robot drive forward for 1 second then stop
    autoChooser.addOption("Drive Forward", swerveSubsystem.driveForward().withTimeout(1));
    
    //Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    DoubleSupplier speedMod = () -> driver.getRawAxis(XboxController.Axis.kRightTrigger.value) == 1 ? 2.5 : 1;
    /* NOTE: the division is used to reduce the speed of the robot when the left trigger is held */
    DoubleSupplier translationX = () -> -MathUtil.applyDeadband(driver.getLeftY(), Constants.Driver.leftStick.Y) / speedMod.getAsDouble();
    DoubleSupplier translationY = () -> -MathUtil.applyDeadband(driver.getLeftX(), Constants.Driver.leftStick.X) / speedMod.getAsDouble();

    /* rotation controls for the robot */
    DoubleSupplier angularRotationX = () -> -MathUtil.applyDeadband(driver.getRawAxis(4), Constants.Driver.rightStick.X) / speedMod.getAsDouble();
    
    spencerAutoAim = new SpencerAutoAim(driver, swerveSubsystem, shootingSubsystem);
    
    /* ----------DRIVER CONTROLS----------- */

    driver.x().toggleOnTrue(spencerAutoAim);

    Command driverControls = swerveSubsystem.driveCommand(translationX, translationY, angularRotationX);
    swerveSubsystem.setDefaultCommand(driverControls);

    driver.y().onTrue(new InstantCommand(swerveSubsystem::zeroGyro));

    /* -------OPERATOR CONTROLS--------- */
    operator.R2().whileTrue(new FeedCommand(shootingSubsystem, 0.5)); // Feed
    operator.options().whileTrue(new FeedCommand(shootingSubsystem, -0.5)); // Reverse Feed (hopefully never used)
    
    operator.R1().whileTrue(new ShootingWheelsCommand(shootingSubsystem, spencerAutoAim)); // Spin up to shoot balls

    operator.L2().whileTrue(new IntakeWheelsCommand(intakeSubsystem, 1)); // Intake balls
    operator.L1().whileTrue(new IntakeWheelsCommand(intakeSubsystem, -0.5)); // Outtake (hopefully never used)
    
    operator.square().onTrue(new SetIntakeCommand(intakeSubsystem, 0.5)); //NOT ACTUAL VALUE (deploy intake)
    operator.circle().onTrue(new SetIntakeCommand(intakeSubsystem, 0)); //NOT ACTUAL VALUE (withdraw intake)

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
