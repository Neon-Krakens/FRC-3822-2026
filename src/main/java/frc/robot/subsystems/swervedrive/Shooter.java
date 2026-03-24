package frc.robot.subsystems.swervedrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;


public class Shooter extends SubsystemBase
{
    boolean enabled = true;

    //Motors
    final SparkMax shooterLeft = new SparkMax(12, MotorType.kBrushless);
    final SparkMax shooterRight = new SparkMax(13, MotorType.kBrushless);
    final SparkMax shooterIntake = new SparkMax(14, MotorType.kBrushless);

    //PID control, see https://github.com/REVrobotics/REVLib-Examples/blob/main/Java/SPARK/Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java
    SparkClosedLoopController shooterController;
    SparkMaxConfig shooterConfig;
    SparkMaxConfig shooterRightConfig;
    RelativeEncoder shooterEncoder; //only for logging/monitoring

    public void Initialize() {
        shooterController = shooterLeft.getClosedLoopController();
        shooterConfig = new SparkMaxConfig();
        shooterRightConfig = new SparkMaxConfig();
        shooterEncoder = shooterLeft.getEncoder();

        // set PID parameters
        shooterConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.0001)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        .feedForward
          // kV is now in Volts, so we multiply by the nominal voltage (12V)
          .kV(12.0 / 5767, ClosedLoopSlot.kSlot1); //says that 12v usually results in 5767 RPM

        shooterRightConfig.follow(12, true); //follows 12, inverts

        shooterLeft.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        shooterRight.configure(shooterRightConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setRPM(double setPoint) {
        if (enabled) {
            shooterController.setSetpoint(setPoint, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        } else {
            shooterController.setSetpoint(0, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        }

        SmartDashboard.putNumber("RPM", shooterEncoder.getVelocity());
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

    public Command enableTopShooter()
    {
        return Commands.run(()->
        {
            enabled = true;
        })
        .finallyDo(()->
        {
            enabled = false;
        });
    }

    public Command shootForward() 
    {
        return Commands.parallel(
            spinShooterIntake(),
            enableTopShooter()
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
            enabled = false;
        });
    }
}
