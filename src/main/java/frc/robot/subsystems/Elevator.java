package frc.robot.subsystems;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPIDConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorSetpointConstants;
import frc.robot.Ports.ElevatorPorts;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;


public class Elevator extends SubsystemBase {

    private final SparkMax elevatorMotorLeader;
    private final SparkMax elevatorMotorFollower;
    private final SparkMaxConfig leaderConfig;
    private final SparkMaxConfig followerConfig;
    private final PIDController elevatorPID;
    private final DigitalInput limitSwitch;
    private final RelativeEncoder relativeEncoder;
    private final AbsoluteEncoder absoluteEncoder;

    public Elevator() {
        elevatorMotorLeader = new SparkMax(ElevatorPorts.ELEVATOR_LEADER_MOTOR, MotorType.kBrushless);
        elevatorMotorFollower = new SparkMax(ElevatorPorts.ELEVATOR_FOLLOWER_MOTOR, MotorType.kBrushless);

        leaderConfig = new SparkMaxConfig();
        leaderConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(ElevatorConstants.CURRENT_LIMIT).inverted(false);
        followerConfig = new SparkMaxConfig();
        followerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(ElevatorConstants.CURRENT_LIMIT).follow(elevatorMotorLeader, true);

        elevatorMotorLeader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorMotorFollower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       
        elevatorPID = new PIDController(ElevatorPIDConstants.kP, ElevatorPIDConstants.kI, ElevatorPIDConstants.kD);
        elevatorPID.setTolerance(0.1);

        limitSwitch = new DigitalInput(ElevatorPorts.LIMIT_SWITCH);

        relativeEncoder = elevatorMotorLeader.getEncoder();
        absoluteEncoder = elevatorMotorLeader.getAbsoluteEncoder();

    }

    public void elevatorPID(double current, double setpoint){
        elevatorMotorLeader.setVoltage(elevatorPID.calculate(current, setpoint));
    }

    public Command runElevatorMotorCmd() {
        return this.run(() -> elevatorMotorLeader.set(ElevatorConstants.ELEVATOR_MOTOR_SPEED));
    }

    public Command reverseElevatorMotorCmd() {
        return this.run(() -> elevatorMotorLeader.set(-ElevatorConstants.ELEVATOR_MOTOR_SPEED));
    }

    public Command stopElevatorMotorCmd() {
        return this.runOnce(() -> elevatorMotorLeader.set(0));
    }

    public Command setVoltageCmd(double voltage) {
        return this.run(() -> elevatorMotorLeader.setVoltage(voltage));
    }

    public Command resetEncoder() {
        return this.runOnce(() -> relativeEncoder.setPosition(0));
    }

    public Command setLevel(double setpoint) {
        return this.run(() -> elevatorPID(relativeEncoder.getPosition(), setpoint));
    }

    public Command setDefaultLevel() {
        return this.run(() -> {
            if (hitLimitSwitch()) {
                elevatorMotorLeader.stopMotor();
                relativeEncoder.setPosition(0);
            } else {
                elevatorMotorLeader.set(-ElevatorConstants.ELEVATOR_MOTOR_SPEED);
            }
            
        });
    }

    public double getPosition() {
        return relativeEncoder.getPosition();
    }

    public boolean hitLimitSwitch() {
        return !limitSwitch.get();
    }
}
