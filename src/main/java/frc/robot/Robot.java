// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{

  private static Robot   instance;
  private        Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();

    if (isSimulation())
    {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    final double latency = .5; //seconds from signal to shoot to exit of ball. Probably this whole section of code should be run once to move motors and a second time after to see if correct, or if further adjustments are needed

    //robot pos, assumes rotational and angular velocity remains constant
    double r = m_robotContainer.drivebase.swerveDrive.getPose().getRotation().getRadians();
    double rv = m_robotContainer.drivebase.swerveDrive.getRobotVelocity().omegaRadiansPerSecond;
    double xv = m_robotContainer.drivebase.swerveDrive.getRobotVelocity().vxMetersPerSecond;
    double yv = m_robotContainer.drivebase.swerveDrive.getRobotVelocity().vyMetersPerSecond;
    double xp = m_robotContainer.drivebase.swerveDrive.getPose().getX()+0.19685*Math.cos(r+latency*rv)+latency*xv; //TODO: check if this rotates correctly
    double yp = m_robotContainer.drivebase.swerveDrive.getPose().getY()+0.19685*Math.sin(r+latency*rv)+latency*yv;
    xv += -0.19685*rv*Math.sin(r+latency*rv); //add on velocity from angular velocity
    yv += 0.19685*rv*Math.cos(r+latency*rv);

    System.out.println("Pose: xp: " + xp + ", yp: " + yp + ", xv:" + xv + ", yv: " + yv);
    
    // position on field, 4.62534 meters X, 4.03479 meters Y
    final double target_x = 4.62534;
    final double target_y = 4.03479;

    //global shooting velocities
    final double target_height = 1.8288;
    final double edge_height = target_height + .3;
    double target_distance = Math.sqrt((target_x-xp)*(target_x-xp)+(target_y-yp)*(target_y-yp));
    double edge_distance = target_distance-.6;
    double x_velocity = Math.sqrt(4.9*target_distance*edge_distance*(target_distance - edge_distance)/(edge_height*target_distance - edge_distance*target_height));
    double z_velocity = x_velocity*target_height/target_distance + 4.9*target_distance/x_velocity;
    
    //local shooting velocities
    double x_target_dir = (target_x - xp)/target_distance;
    double y_target_dir = (target_y - yp)/target_distance;
    double x_rel_velocity = x_target_dir * x_velocity - xv;
    double y_rel_velocity = y_target_dir * x_velocity - yv;
    double shooting_vel_xy = Math.sqrt(x_rel_velocity*x_rel_velocity + y_rel_velocity*y_rel_velocity);
    double shooting_dir_horiz = Math.atan2(y_rel_velocity, x_rel_velocity);
    double shooting_dir_virt = Math.atan2(z_velocity, shooting_vel_xy);
    double shooting_speed = Math.sqrt(shooting_vel_xy*shooting_vel_xy + z_velocity*z_velocity);

    System.out.println("Targeting: shooting_speed: " + shooting_speed + ", shooting_dir_horiz: " + shooting_dir_horiz + ", shooting_dir_virt:" + shooting_dir_virt);
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }
  
  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
      disabledTimer.reset();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    //Print the selected autonomous command upon autonomous init
    System.out.println("Auto selected: " + m_autonomousCommand);

    // schedule the autonomous command selected in the autoChooser
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    } else
    {
      CommandScheduler.getInstance().cancelAll();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {
  }

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
  }
}
