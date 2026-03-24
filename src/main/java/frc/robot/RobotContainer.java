package frc.robot;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.subsystems.swervedrive.Actuator;
// import frc.robot.subsystems.swervedrive.Agitator;
// import frc.robot.subsystems.swervedrive.Intake;
// import frc.robot.subsystems.swervedrive.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
// import frc.robot.subsystems.swervedrive.Turret;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.io.File;
import com.pathplanner.lib.auto.NamedCommands;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.swervedrive.Vision;
import com.pathplanner.lib.commands.PathPlannerAuto;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{


    final CommandXboxController driverXbox = new CommandXboxController(0);
    final CommandXboxController shooterXbox = new CommandXboxController(1);

    //The robot's subsystems and commands are defined here...
    public final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));


    /**
    * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
    */
    public SwerveInputStream driveInputStream = SwerveInputStream.of(drivebase.getSwerveDrive(),
    () -> driverXbox.getLeftY() * 1,
    () -> driverXbox.getLeftX() * 1)
      .withControllerHeadingAxis(() -> driverXbox.getRightX() * 1, () -> driverXbox.getRightY() * 1) //TODO: controllerrotationaxis for relative
      .deadband(Constants.OperatorConstants.DEADBAND)
      .scaleTranslation(0.5)
      .allianceRelativeControl(false);
      //.headingWhile(true);
      
  // //This is robot oriented driving. Don't touch it.
  //   public SwerveInputStream driveDirectAngle = driveInputStream.copy()
  //                       .withControllerHeadingAxis(() -> driverXbox.getRightX()*-1,
  //                                       () -> driverXbox.getRightY()*-1)
  //                       .headingWhile(true);

    /**
    * The container for the robot. Contains subsystems, OI devices, and commands.
    */
    public RobotContainer()
    {

      //Configure the PathPlanner commands
      setupPathPlannerCommands();

      //Configure the trigger bindings
      configureBindings();

      DriverStation.silenceJoystickConnectionWarning(true);
    }

    private void setupPathPlannerCommands()
    {
    }

   private void configureBindings()
    {
      driverXbox.b().onTrue(Commands.runOnce(drivebase::zeroGyro));

      Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveInputStream);

      //Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
     
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

      /****************************************************** Shooter Commands ******************************************************/
      //Top Shooter: Toggle On and Off
      // //Y = up Actuator
      // shooterXbox.y().onTrue(actuator.goUpCommand());
      //
      // //A = down Actuator
      // shooterXbox.a().onTrue(actuator.goDownCommand());

      //TODO: add manual turret and shooter control mode
      // //Movement Left
      // shooterXbox.povLeft()
      //   .whileTrue(Commands.run(() -> turret.testTurnLeft(), turret))
      //   .onFalse(Commands.runOnce(() -> turret.stopTurret(), turret));
      //
      // //Turret: Movement Right
      // shooterXbox.povRight()
      //   .whileTrue(Commands.run(() -> turret.testTurnRight(), turret))
      //   .onFalse(Commands.runOnce(() -> turret.stopTurret(), turret));

      //TODO: reimplement shooter reverse
      //Agitator and shooter intake reverse
      //shooterXbox.x()
      //  .whileTrue(Commands.parallel(agitator.funnelReverse(), shooter.shooterIntakeReverse()))
      //  .onFalse(Commands.parallel(agitator.funnelStop(), shooter.stopShooterIntake()));

        
      /****************************************************************************************************************************/

      /************************************************* Driver Commands *************************************************/
      // //Turret: Movement Left
      // driverXbox.b()
      //   .whileTrue(Commands.run(() -> turret.testTurnLeft(), turret))
      //   .onFalse(Commands.runOnce(() -> turret.stopTurret(), turret));

      // //Turret: Movement Right
      // driverXbox.a()
      //   .whileTrue(Commands.run(() -> turret.testTurnRight(), turret))
      //   .onFalse(Commands.runOnce(() -> turret.stopTurret(), turret));
      /******************************************************************************************************************/
       
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
      System.out.println("Working!");
      // return new PathPlannerAuto("Middle Auto");
      // return drivebase.runOnce(() -> new ChassisSpeeds(MetersPerSecond.of(-3), MetersPerSecond.of(0), RadiansPerSecond.of(0))).withTimeout(1.95).andThen(intake.foldOpenIntake());
      return drivebase.driveForward().withTimeout(1.95);

    }

    public void setMotorBrake(boolean brake)
    {
      drivebase.setMotorBrake(brake);
    }

    /************************************************************** Command **************************************************************/
  
}
