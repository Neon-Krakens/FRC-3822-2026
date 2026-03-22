package frc.robot.subsystems.swervedrive;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;

public class Turret extends SubsystemBase
{
    //Motor
    private final SparkMax turret = new SparkMax(11, MotorType.kBrushless);

    //Encoder
    private final RelativeEncoder turretEncoder = turret.getEncoder();

    //TEMP LIMITS. SUBJECT TO CHANGE AFTER TESTING
    private static final double MIN_TURRET_POSITION = 0.0;
    private static final double MAX_TURRET_POSITION = 0.55;

    //PID for aiming
    private final PIDController aimPID = new PIDController(0.02, 0.0, 0.0);

     public Turret()
    {
        aimPID.setTolerance(.01);
    }

    public void setTurretPower(double power)
    {
        double position = getTurretAngle();

        //STOP if trying to go past left limit
        if (position <= MIN_TURRET_POSITION && power < 0)
        {
            turret.set(0.0);
            return;
        }

        //STOP if trying to go past right limit
        if (position >= MAX_TURRET_POSITION && power > 0)
        {
            turret.set(0.0);
            return;
        }
        
        turret.set(MathUtil.clamp(power, -0.35, 0.35)); //TODO: evaluate clamp (motor can go to -1, 1)
    }

    public void stopTurret()
    {
        turret.set(0.0);
    }

        public void testTurnLeft()
    {
        setTurretPower(-0.15);
    }

    public void testTurnRight()
    {
        setTurretPower(0.15);
    }

    public void aimAtTarget(double angle)
    {
        double output = aimPID.calculate(angle, 0.0);
        setTurretPower(output);
    }

    public boolean aimedAtTarget()
    {
        return aimPID.atSetpoint();
    }

    public double getTurretAngle()
    {
        //note: large gear to small is 10:1, neo 4:1
        return turretEncoder.getPosition()/(40) - 1.74; //TODO: determine what angle is actually correct for 0
    }

    @Override
    public void periodic()
    {
        //System.out.println("Turret Position: " + turretEncoder.getPosition());
    } 

}
