package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants9752.ArmConstants;
import frc.robot.commands.AutoPivotAndShoot;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SmartOuttake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GateSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandPS5Controller base = new CommandPS5Controller(0);

  private final LoggedDashboardChooser<Command> autoChooser;

  private final Drive drive;
  public static final TransferSubsystem transfer = new TransferSubsystem();
  private static final GateSubsystem gate = new GateSubsystem();
  public static final IntakeSubsystem intake = new IntakeSubsystem();
  private static final ShooterSubsystem shooter = new ShooterSubsystem();
  public static final ArmSubsystem arm = new ArmSubsystem();
  public static final LimelightSubsystem limelight = new LimelightSubsystem();
  private static final LEDSubsystem LEDs = new LEDSubsystem();

  public RobotContainer() {

    switch (Constants9752.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(true),
                new ModuleIOTalonFX(0), // fl
                new ModuleIOTalonFX(1), // fr
                new ModuleIOTalonFX(2), // bl
                new ModuleIOTalonFX(3)); // br
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    DogLog.setOptions(new DogLogOptions().withNtPublish(true));
    DogLog.setPdh(new PowerDistribution());
    DogLog.log("RobotContainer/Constructor", "Init");
    configureDefaultCommands();
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    base.square().onTrue(Commands.runOnce(drive::stopWithX, drive));
    base.circle()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    base.L2().whileTrue(new SmartOuttake(intake, transfer, gate, arm));
    base.R1().whileTrue(new ShootCommand(transfer, shooter, gate, 5000, 6000));
    base.cross().onTrue(arm.SetArmToPos(ArmConstants.ampPos));
    base.touchpad().onTrue(arm.SetArmToPos(ArmConstants.subPos));
    base.triangle().onFalse(new AutoPivotAndShoot(arm, shooter, gate, transfer, limelight));
  }

  public void configureDefaultCommands() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -base.getLeftY(),
            () -> -base.getLeftX(),
            () -> -base.getRightX(),
            () -> -base.getRightY()));

    intake.setDefaultCommand(intake.DefaultCommand());
    transfer.setDefaultCommand(transfer.DefaultCommand());
    shooter.setDefaultCommand(shooter.DefaultCommand());
    LEDs.setDefaultCommand(LEDs.defaultCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
