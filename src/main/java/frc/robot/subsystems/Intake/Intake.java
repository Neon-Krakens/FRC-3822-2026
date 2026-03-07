package frc.robot.subsystems.Intake;

import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.Constants;

public class Intake {
    SparkMax rotationMotor = new SparkMax(Constants.INTAKE_ROTATION_MOTOR_ID, MotorType.kBrushless);
    SparkClosedLoopController rotationMotorController = rotationMotor.getClosedLoopController();
    RelativeEncoder rotationMotorEncoder = rotationMotor.getEncoder();


/**
    ** Initializes intake rotation motor and registers commands `Go to Zero`, and `Go to 90`.
    **/
    public void initialize() {

        SparkMaxConfig config = new SparkMaxConfig();

        config.encoder
            .positionConversionFactor(360.0)
            .velocityConversionFactor(360.0 / 60.0); //degrees per second

        config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);


        config.closedLoop
            .p(0.1)
            .i(0.0)
            .d(0.0)
            .velocityFF(0.0)
            .outputRange(-0.5,0.5);

        rotationMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); 

        rotationMotorEncoder.setPosition(90.0);

        NamedCommands.registerCommand("GoToZero",
            Commands.runOnce(() -> rotationMotorController.setReference(0.0, SparkMax.ControlType.kPosition)));
        NamedCommands.registerCommand("GoTo90",
            Commands.runOnce(() -> rotationMotorController.setReference(90.0, SparkMax.ControlType.kPosition)));
    }
}
