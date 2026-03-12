package frc.subsystem;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.io.IOException;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Constants;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * <h2>Swerve Subsystem</h2>
 * 
 * <p>
 * Represents all four modules present on the drivetrain.
 * </p>
 * 
 * Code taken from:
 * {@link https://github.com/Yet-Another-Software-Suite/YAGSL/blob/main/examples/full_example/src/main/java/frc/robot/subsystems/swervedrive/SwerveSubsystem.java}
 * 
 */
public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;
    private static SwerveSubsystem instance;

    // Path Planning Paths
    public PathPlannerPath  Z1R = loadIndividualPath("1R"),
                            Z1L = loadIndividualPath("1L"),
                            Z2R = loadIndividualPath("2R"),
                            Z2L = loadIndividualPath("2L"),
                            Z3R = loadIndividualPath("3R"),
                            Z3L = loadIndividualPath("3L"),
                            Z4R = loadIndividualPath("4R"),
                            Z4L = loadIndividualPath("4L"),
                            Z5R = loadIndividualPath("5R"),
                            Z5L = loadIndividualPath("5L"),
                            Z6R = loadIndividualPath("6R"),
                            Z6L = loadIndividualPath("6L");

    public SwerveSubsystem() {
        try {
            this.swerveDrive = new SwerveParser(
                    new File(
                            Filesystem.getDeployDirectory(), "swerve"))
                    .createSwerveDrive(Constants.SWERVE_MAXIMUM_SPEED.in(MetersPerSecond));
        } catch (Exception e) {
            throw new RuntimeException("Failed to create swerve drive ", e);
        }

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        swerveDrive.setHeadingCorrection(false);
        swerveDrive.setAngularVelocityCompensation(
            true, 
            true, 
            0.1
            );
        swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);
        swerveDrive.setModuleEncoderAutoSynchronize(true, 1);
        swerveDrive.pushOffsetsToEncoders();
        swerveDrive.setMaximumAllowableSpeeds(Constants.SWERVE_MAXIMUM_SPEED.in(MetersPerSecond), Constants.SWERVE_MAXIMUM_ANGULAR_VELOCITY);
        
    }

    /**
     * Get the chassis speeds based on controller input of 2 joysticks. One for
     * speeds in which direction. The other for
     * the angle of the robot.
     *
     * @param xInput   X joystick input for the robot to move in the X direction.
     * @param yInput   Y joystick input for the robot to move in the Y direction.
     * @param headingX X joystick which controls the angle of the robot.
     * @param headingY Y joystick which controls the angle of the robot.
     * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
     */
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
        return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                scaledInputs.getY(),
                headingX,
                headingY,
                getHeading().getRadians(),
                Units.feetToMeters(14.5));
    }

    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> {
            swerveDrive.driveFieldOriented(velocity.get());
        });
    }

    /**
     * Drive according to the chassis robot oriented velocity.
     *
     * @param velocity Robot oriented {@link ChassisSpeeds}
     */
    public void drive(ChassisSpeeds velocity) {
        swerveDrive.drive(velocity);
    }

    /**
     * Scoot the robot left.
     * @return
     */
    public Command scootLeft() {
        return run(() -> {
            swerveDrive.drive(new Translation2d(0, 0.5), 0, false, false);
        }); // Adjust time as needed
    }

    /**
     * Scoot the robot right.
     * @return
     */
    public Command scootRight() {
        return run(() -> {
            swerveDrive.drive(
                new Translation2d(
                    0, 
                    -0.5
                ), 0, false, false);
        }); // Adjust time as needed
    }

    /**
     * Scoot the robot backwards.
     * @return
     */
    public Command scootBackward() {
        return runOnce(() -> {
            swerveDrive.drive(new Translation2d(-0.5, 0.0), 0, false, false);
        }); // Adjust time as needed
    }

    /**
     * Resets the robot's odometry to the center of the field (8.774m, 4.026m, 0°).
     * This is typically used at the start of autonomous routines.
     */
    public void resetOdometry() {
        swerveDrive.zeroGyro();
        swerveDrive.resetOdometry(new Pose2d(new Translation2d(Meter.of(8.774), Meter.of(4.026)),Rotation2d.fromDegrees(0)));
    }

    public PathPlannerPath loadIndividualPath(String pathName) {
        try {
            PathPlannerPath.fromPathFile(pathName);
        } catch (FileVersionException | IOException | ParseException e) {
            e.printStackTrace();
        }
        return null;
    }

    /**
     * Returns the singleton instance of the Swerve subsystem.
     * Creates a new instance if one does not exist.
     * 
     * @return the Swerve subsystem instance
     */
    public static SwerveSubsystem getInstance() {
        if (instance == null) {
            instance = new SwerveSubsystem();
        }
        return instance;
    }

    /**
     * Gets the underlying SwerveDrive object.
     * 
     * @return the SwerveDrive instance used by this subsystem
     */
    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    /**
     * Retrieves the current estimated pose of the robot on the field.
     * 
     * @return the current Pose2d representing the robot's position and rotation
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * Gets the current yaw angle of the robot, as reported by the swerve pose
     * estimator in the underlying drivebase.
     * Note, this is not the raw gyro reading, this may be corrected from calls to
     * resetOdometry().
     *
     * @return The yaw angle
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }
}
