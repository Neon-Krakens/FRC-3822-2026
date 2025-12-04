package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Bot.Swerve;
import swervelib.SwerveInputStream;
import frc.robot.Constants;

/**
 * Main robot configuration class that binds controls and commands to
 * subsystems.
 * This class serves as the robot's command center, managing all subsystem
 * instances
 * and their associated commands.
 * 
 * <p>
 * Features include:
 * <ul>
 * <li>Driver control configuration
 * <li>Command button mappings
 * <li>Autonomous command selection
 * <li>Subsystem instantiation and management
 * </ul>
 * 
 * <p>
 * The class follows a centralized control pattern, with all robot behaviors
 * defined through command bindings and default commands.
 */
public class RobotContainer {

  /** Xbox controller used for driver input. */
  private final CommandXboxController xboxController = new CommandXboxController(0);
  private final SendableChooser<String> autos = new SendableChooser<>();
  /** Main drive subsystem for robot movement. */
  private final Swerve swerveDrive = Swerve.getInstance();

  /**
   * Input stream for swerve drive control.
   * Configures how controller inputs are processed and applied to drive commands.
   */
  public SwerveInputStream driveInputStream = SwerveInputStream.of(swerveDrive.getSwerveDrive(),
      () -> xboxController.getLeftY() * 1,
      () -> xboxController.getLeftX() * 1)
      .withControllerHeadingAxis(() -> xboxController.getRightX() * 1, () -> xboxController.getRightY() * 1)
      .deadband(Constants.DRIVER_DEADBAND)
      .scaleTranslation(0.5)
      .allianceRelativeControl(true)
      .headingWhile(true);

  /**
   * Creates a new RobotContainer and initializes all robot subsystems and
   * commands.
   * Performs the following setup:
   * <ul>
   * <li>Silences joystick warnings for unplugged controllers
   * <li>Disables controller rumble
   * <li>Configures button bindings for commands
   * </ul>
   */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    xboxController.setRumble(RumbleType.kBothRumble, 0.0);

    configureBindings();
    setupPathPlannerCommands();

    autos.addOption("Left2", "LEFT2");
    autos.setDefaultOption("Middle", "MID");
    autos.addOption("Right2", "RIGHT2");

    // Add the chooser to Shuffleboard
    SmartDashboard.putData("Auto Chooser", autos);

    CommandScheduler.getInstance().onCommandInitialize(command -> {
      System.out.println("Command Started: " + command.getName());
    });

    CommandScheduler.getInstance().onCommandInterrupt(command -> {
      System.out.println("Command Interrupted: " + command.getName());
    });

    CommandScheduler.getInstance().onCommandFinish(command -> {
      System.out.println("Command Finished: " + command.getName());
    });
  }

  private void setupPathPlannerCommands() {
    // Commands for PathPlanner auto routines
    NamedCommands.registerCommand("Move Back", swerveDrive.scootBackward());
  }

  /**
   * Configures button bindings for commands.
   * Maps controller buttons to specific robot actions:
   * <ul>
   * <li>Default command: Field oriented drive using joystick input
   * <li>X button: Lock wheels in X pattern for stability
   * <li>Start button: Reset odometry to field center
   * <li>Back button: Autonomous drive to field center
   * </ul>
   */
  private void configureBindings() {
    Command driveFieldOrientedDirectAngle = swerveDrive.driveFieldOriented(driveInputStream);
    swerveDrive.setDefaultCommand(driveFieldOrientedDirectAngle);

    xboxController.start().onTrue(Commands.runOnce(swerveDrive::resetOdometry, swerveDrive));

    // xboxController.y().toggleOnTrue(swerveDrive.cancelPathfinding());
    // xboxController.x().toggleOnTrue(swerveDrive.goToClosestCoralTag(true));
    // xboxController.b().toggleOnTrue(swerveDrive.goToClosestCoralTag(false));
    
    xboxController.povLeft().whileTrue(swerveDrive.scootLeft());
    xboxController.povRight().whileTrue(swerveDrive.scootRight());
  }

  /**
   * Provides the command to run during autonomous mode.
   * Currently returns a placeholder command that prints a message,
   * indicating no autonomous routine is configured.
   *
   * @return the command to run in autonomous mode
   */
  public Command getAutonomousCommand() {
    String selectedAuto = autos.getSelected();

    switch (selectedAuto) {
      case "LEFT2":
        return new PathPlannerAuto("Left Auto 2 Coral");
      case "MID":
        return new PathPlannerAuto("Mid Auto");
      case "RIGHT2":
        return new PathPlannerAuto("Right Auto 2 Coral");
      default:
        break;
    }

    return new PathPlannerAuto("Mid Auto");
  }

  public CommandXboxController getController() {
    return xboxController;
  }
}