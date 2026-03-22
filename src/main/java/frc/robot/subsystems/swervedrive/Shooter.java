package frc.robot.subsystems.swervedrive;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase
{
    //Motors
    final SparkMax shooterLeft = new SparkMax(12, MotorType.kBrushless);
    final SparkMax shooterRight = new SparkMax(13, MotorType.kBrushless);
    final SparkMax shooterIntake = new SparkMax(14, MotorType.kBrushless);

    //PID control, see https://github.com/REVrobotics/SPARK-MAX-Examples/blob/b519a006fbc2ce769eaba191db442e59714e255a/Java/Velocity%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java
    final SparkPIDController shooterPID;
    final RelativeEncoder shooterEncoder; //only for logging/monitoring

    public void Initialize() {
        shooterPID = shooterLeft.getPIDController();
        shooterEncoder = shooterLeft.getEncoder();

        // set PID coefficients
        shooterPID.setP(6e-5);
        shooterPID.setI(0);
        shooterPID.setD(0);
        shooterPID.setIZone(0);
        shooterPID.setFF(0.000015);
        shooterPID.setOutputRange(-1, 1);
    }

    public void setRPM(setPoint) {
        shooterPID.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
        SmartDashboard.putNumber("RPM", m_encoder.getVelocity());
        SmartDashboard.putNumber("RPM Target", setPoint);
    }
   
    //************************************************* Commands *************************************************/
    public Command spinShooterIntake()
    {
        return Commands.runOnce(()->
        {
            shooterIntake.set(-100.0);
        });
    }

    public Command stopShooterIntake()
    {
        return Commands.runOnce(()->
        {
            shooterIntake.set(0.0);
        });
    }
    
    public Command spinTopShooterReverse() 
    {
        return Commands.runOnce(()->
        {
            shooterLeft.set(100.0);
            shooterRight.set(-100.0);
        });
    }

    public Command shooterIntakeReverse()
    {
        return Commands.runOnce(()->
        {
            shooterIntake.set(100);
        });
       
    }

    public Command spinTopShooter() 
    {
        return Commands.run(()->
        {
            shooterLeft.set(100.0);
            shooterRight.set(-100.0);
        })
        .finallyDo(()->
        {
            shooterLeft.set(0);
            shooterRight.set(0);
        });
    }

    public Command shootForward() 
    {
        return Commands.parallel(
            spinShooterIntake(),
            spinTopShooter()
        );
    }

    public Command shootStop() 
    {
        return Commands.parallel(
            stopShooterIntake(),
            stopTopShooter()
        );
    }

    public Command stopTopShooter() 
    {
        return Commands.runOnce(()->
        {
            shooterLeft.set(0.0);
            shooterRight.set(0.0);
        });
    }

    public Command goToLaunchAngle(double angle) 
    {
        return Commands.runOnce(()->{});
    }

    public Command turnToAngle(double angle) 
    {
        return Commands.runOnce(()->{});
    }
}
