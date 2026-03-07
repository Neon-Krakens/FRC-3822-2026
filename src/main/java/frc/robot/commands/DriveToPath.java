package frc.robot.commands;


import java.util.function.BooleanSupplier;


import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystem.SwerveSubsystem;

/** Command to pathfind to a prebuilt path and follow it */
public class DriveToPath extends Command {
  private Command pathFollowingCommand;
  // private final PathConstraints constraints;
  private final BooleanSupplier interupter;

  /** Creates a new DriveToPoint with a prebuilt path and default constraints */
  public DriveToPath(SwerveSubsystem drive, PathPlannerPath path) {
    this.interupter = null;

    if (path == null) {
      throw new IllegalArgumentException("Path cannot be null");
    }

    // Create path constraints with default values
    // this.constraints =
    // new PathConstraints(
    //     Constants.MAX_SPEED.in(MetersPerSecond) * 0.5, // 50% of max velocity
    //     Constants.MAX_SPEED.in(MetersPerSecond) * 0.4, // 50% of max acceleration
    //     Constants.MAX_ANGULAR_VELOCITY * 0.7, // 70% of max angular velocity
    //     Constants.MAX_ANGULAR_VELOCITY * 0.7 // 70% of max angular acceleration
    // );

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Create pathfinding command to the prebuilt path
    // pathFollowingCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);

    // Schedule the path following command
    pathFollowingCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Update current status
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (pathFollowingCommand != null) {
      pathFollowingCommand.cancel();
    }

    // Log final status
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (interupter != null && interupter.getAsBoolean()) {
      return true;
    }
    return pathFollowingCommand != null && pathFollowingCommand.isFinished();
  }
}