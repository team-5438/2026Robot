// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.SetIntakeCommand;
import frc.robot.commands.AutoAdjustingShootyWheels;
import frc.robot.commands.FeedCommand;
import frc.robot.commands.HungryHippo;
import frc.robot.commands.IntakeWheelsCommand;
import frc.robot.commands.InterpolatingAutoAim;
import frc.robot.commands.ManualHoodCommand;
import frc.robot.commands.ManualIntakeCommand;
import frc.robot.commands.MusicCommand;
import frc.robot.commands.SetHoodCommand;
import frc.robot.commands.ShootingWheelsCommand;
import frc.robot.commands.SpencerAutoAlign;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import gg.questnav.questnav.QuestNav;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
  public QuestNavSubsystem questNavSubsystem;
  public IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public final CommandXboxController driver = new CommandXboxController(Constants.Driver.id);
  public final CommandPS5Controller operator = new CommandPS5Controller(Constants.Operator.id);
  public SpencerAutoAlign spencerAutoAlign;
  public InterpolatingAutoAim interpolatingAutoAim;

  private final SendableChooser<Command> autoChooser;
  public final SendableChooser<Chooser> initialChooser;
  public final SendableChooser<String> musicChooser;
  public MusicCommand musicCommand;

  
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
                                                                () -> -driver.getLeftY(),
                                                                () -> -driver.getLeftX())
                                                            .withControllerRotationAxis( () -> -driver.getRightX())
                                                            .deadband(0.1)
                                                            .scaleTranslation(1.0)
                                                            .scaleRotation(1)
                                                            .allianceRelativeControl(true)
                                                            .robotRelative(false);

  SwerveInputStream driveAngularvelocitySlow = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
                                                                () -> -driver.getLeftY(),
                                                                () -> -driver.getLeftX())
                                                            .withControllerRotationAxis( () -> -driver.getRightX())
                                                            .deadband(0.1)
                                                            .scaleTranslation(0.4)
                                                            .scaleRotation(0.4)
                                                            .allianceRelativeControl(true)
                                                            .robotRelative(false);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    musicChooser = setUpMusicChooser();
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    
    //Create the NamedCommands that will be used in PathPlanner
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("spin up", new AutoAdjustingShootyWheels(shootingSubsystem, swerveSubsystem, interpolatingAutoAim, 0.925));
    NamedCommands.registerCommand("shoot", new FeedCommand(shootingSubsystem, 1));
    NamedCommands.registerCommand("auto aim", new InterpolatingAutoAim(shootingSubsystem, swerveSubsystem, operator));
    NamedCommands.registerCommand("intake", new IntakeWheelsCommand(intakeSubsystem, 1));
    NamedCommands.registerCommand("deploy intake", new SetIntakeCommand(intakeSubsystem, 0.075));
    NamedCommands.registerCommand("withdraw intake", new SetIntakeCommand(intakeSubsystem, 0.305));
    NamedCommands.registerCommand("hungry hippo", new HungryHippo(intakeSubsystem));

    //Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();

    //Set the default auto (do nothing) 
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    //Add a simple auto option to have the robot drive forward for 1 second then stop
    autoChooser.addOption("Drive Forward", swerveSubsystem.driveForward().withTimeout(1));

    //Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);

    initialChooser = setUpInitialChooser();
    vision = new Vision(swerveSubsystem::getPose, swerveSubsystem.swerveDrive.field);
    questNavSubsystem = new QuestNavSubsystem(swerveSubsystem, vision, initialChooser);

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
    // BELOW IS NOT USED ANYMORE
    // DoubleSupplier speedMod = () -> driver.getRawAxis(XboxController.Axis.kRightTrigger.value) == 1 ? 0.4 : 1;
    // /* NOTE: the division is used to reduce the speed of the robot when the left trigger is held */
    // DoubleSupplier translationX = () -> -MathUtil.applyDeadband(driver.getLeftY(), Constants.Driver.leftStick.Y) / speedMod.getAsDouble();
    // DoubleSupplier translationY = () -> -MathUtil.applyDeadband(driver.getLeftX(), Constants.Driver.leftStick.X) / speedMod.getAsDouble();

    // /* rotation controls for the robot */
    // DoubleSupplier angularRotationX = () -> -MathUtil.applyDeadband(driver.getRawAxis(4), Constants.Driver.rightStick.X) / speedMod.getAsDouble();
    
    ShuffleboardTab autoAlignShuffleboardTab = Shuffleboard.getTab("Auto Aim/Align");
    GenericEntry autoAlignEntry = autoAlignShuffleboardTab.add("Auto Align On", false).getEntry();    
    spencerAutoAlign = new SpencerAutoAlign(driver, swerveSubsystem, autoAlignEntry);

    GenericEntry autoAimEntry = autoAlignShuffleboardTab.add("Auto Aim On", false).getEntry();
    interpolatingAutoAim = new InterpolatingAutoAim(shootingSubsystem, swerveSubsystem, operator);

    musicCommand = new MusicCommand(intakeSubsystem, shootingSubsystem, swerveSubsystem, getSelectedMusic());


    /* ----------DRIVER CONTROLS----------- */

    driver.x().toggleOnTrue(spencerAutoAlign);

    swerveSubsystem.setDefaultCommand(swerveSubsystem.driveFieldOriented(driveAngularVelocity));
    driver.rightTrigger().onTrue(Commands.runOnce(() -> driveAngularVelocity.scaleRotation(0.4).scaleTranslation(0.4)))
                          .onFalse(Commands.runOnce(() -> driveAngularVelocity.scaleRotation(1).scaleTranslation(1)));
    
    driver.y().onTrue(new InstantCommand(swerveSubsystem::zeroGyro));
    driver.b().onTrue(new InstantCommand(shootingSubsystem::resetHoodEncoder));

    // driver.back().toggleOnTrue(musicCommand);

    driver.a().toggleOnTrue(interpolatingAutoAim);


    /* -------OPERATOR CONTROLS--------- */
    operator.R1().whileTrue(new FeedCommand(shootingSubsystem, 1)); // Feed
    operator.options().whileTrue(new ParallelCommandGroup(new FeedCommand(shootingSubsystem, -0.5), new ShootingWheelsCommand(shootingSubsystem, spencerAutoAlign, -0.5))); // Reverse Feed (hopefully never used)
    
    operator.R2().whileTrue(new AutoAdjustingShootyWheels(shootingSubsystem, swerveSubsystem, interpolatingAutoAim, 0.91)); // Spin up to shoot balls
    // operator.R3().whileTrue(new ShootingWheelsCommand(shootingSubsystem, spencerAutoAlign, 0.05)); // Spin up to shoot balls
    // driver.leftTrigger().whileTrue(new ShootingWheelsCommand(shootingSubsystem, spencerAutoAim, 1)); // Spin up to shoot balls
    // driver.leftBumper().whileTrue(new ShootingWheelsCommand(shootingSubsystem, spencerAutoAim, 0.95)); // Spin up to shoot balls
    // operator.L3().whileTrue(new IntakeWheelsCommand(intakeSubsystem, 0.1));

    operator.L2().whileTrue(new IntakeWheelsCommand(intakeSubsystem, 1)); // Intake balls
    operator.L1().whileTrue(new IntakeWheelsCommand(intakeSubsystem, -0.5)); // Outtake (hopefully never used)
    
    operator.cross().onTrue(new SetIntakeCommand(intakeSubsystem, 0.055)); //(deploy intake)
    operator.square().onTrue(new SetIntakeCommand(intakeSubsystem, 0.305)); //(withdraw intake)

    operator.povRight().whileTrue(new ManualIntakeCommand(intakeSubsystem, 0.15)); //manual intake out
    operator.povLeft().whileTrue(new ManualIntakeCommand(intakeSubsystem, -0.1)); //manual intake in

    operator.povDown().whileTrue(new ManualHoodCommand(shootingSubsystem, 0.08)); //manual hood down
    operator.povUp().whileTrue(new ManualHoodCommand(shootingSubsystem, -0.08)); //manual hood up

    operator.circle().toggleOnTrue(new SetHoodCommand(shootingSubsystem, operator)); //set hood to a positoin for shooting back from neutral zone

    // operator.triangle().toggleOnTrue(interpolatingAutoAim); //toggle Auto Aim on/off
    
    operator.touchpad().toggleOnTrue(new HungryHippo(intakeSubsystem)); //intake goes up and down to try and free balls for shooting

    
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

  public enum Chooser {
    REDLEFT(new Pose2d(12.929, 0.419, new Rotation2d(Math.PI))),
    REDRIGHT(new Pose2d(12.929, 7.641,new Rotation2d(Math.PI))),
    
    BLUELEFT(new Pose2d(3.661, 7.641, new Rotation2d(0))),
    BLUERIGHT(new Pose2d(3.661, 0.419, new Rotation2d(0))),
    
    BLUELEFTNEW(new Pose2d(4.460, 7.641, new Rotation2d(-Math.PI/2))),
    BLUERIGHTNEW(new Pose2d(4.460, 0.419, new Rotation2d(Math.PI/2))),
    
    REDLEFTNEW(new Pose2d(12.13, 0.419, new Rotation2d(Math.PI/2))),
    REDRIGHTNEW(new Pose2d(12.13, 7.641, new Rotation2d(-Math.PI/2))),

    CAMERAS(null);

    private Pose2d initialPose;

    private Chooser(Pose2d initialPose){
      this.initialPose = initialPose;
    }

    public Pose2d getInitialPose(Vision vision){
      //might check cameras before cameras actually start (---------IMPORTANT---------)
      if(initialPose == null){
        try{
          return vision.getEstimatedGlobalPose(Vision.Cameras.FrontLeft).get().estimatedPose.toPose2d();
        } catch(Exception E){
          return Robot.isRedAlliance ? new Pose2d(12.97, 4.03, new Rotation2d(Math.PI)) : new Pose2d(4.08, 4.03, new Rotation2d(0));
        }
      }
      return initialPose;
    }
  }
  public SendableChooser<Chooser> setUpInitialChooser(){
    SendableChooser<Chooser> chooser = new SendableChooser<>();
    chooser.addOption("redLeft", Chooser.REDLEFT);
    chooser.addOption("redRight", Chooser.REDRIGHT);
    chooser.addOption("blueLeft", Chooser.BLUELEFT);
    chooser.addOption("blueRight", Chooser.BLUERIGHT);
    chooser.addOption("redLeft NEW", Chooser.REDLEFTNEW);
    chooser.addOption("redRight NEW", Chooser.REDRIGHTNEW);
    chooser.addOption("blueLeft NEW", Chooser.BLUELEFTNEW);
    chooser.addOption("blueRight NEW", Chooser.BLUERIGHTNEW);
    chooser.addOption("cameras", Chooser.CAMERAS);
    chooser.setDefaultOption("cameras", Chooser.CAMERAS);
    SmartDashboard.putData("Initial Position Chooser", chooser);
    return chooser;
  }

  public SendableChooser<String> setUpMusicChooser(){
    SendableChooser<String> musicChooser = new SendableChooser<>();
    musicChooser.addOption("clear sky", "chrp_files/clear_sky.chrp");
    musicChooser.addOption("bad piggies", "chrp_files/badpiggies.chrp");
    musicChooser.addOption("we r the champions", "chrp_files/champions.chrp");
    musicChooser.addOption("stal", "chrp_files/stal.chrp");
    SmartDashboard.putData("Music Chooser", musicChooser);
    return musicChooser;
  }

  public String getSelectedMusic(){
    return musicChooser.getSelected();
  }
}
