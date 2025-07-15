package frc.robot.subsystems;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPIDConstants;
import frc.robot.Ports.ElevatorPorts;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;


public class Elevator extends SubsystemBase {

    private final SparkMax elevatorMotor;
    private final PIDController elevatorPID;
    private final DigitalInput limitSwitch;
    private final RelativeEncoder relativeEncoder;
    private final AbsoluteEncoder absoluteEncoder;

    public Elevator() {
        elevatorMotor = new SparkMax(ElevatorPorts.ELEVATOR_MOTOR, MotorType.kBrushless);
       
        elevatorPID = new PIDController(ElevatorPIDConstants.kP, ElevatorPIDConstants.kI, ElevatorPIDConstants.kD);

        limitSwitch = new DigitalInput(ElevatorPorts.LIMIT_SWITCH);

        relativeEncoder = elevatorMotor.getEncoder();
        absoluteEncoder = elevatorMotor.getAbsoluteEncoder();

    }

    public Command runElevatorMotorCmd() {
        return this.run(() -> elevatorMotor.set(ElevatorConstants.ELEVATOR_MOTOR_SPEED));
    }

    public Command reverseElevatorMotorCmd() {
        return this.run(() -> elevatorMotor.set(-ElevatorConstants.ELEVATOR_MOTOR_SPEED));
    }

    public Command stopElevatorMotorCmd() {
        return this.run(() -> elevatorMotor.set(0));
    }

    public Command setVoltageCmd(double voltage) {
        return this.run(() -> elevatorMotor.setVoltage(voltage));
    }

    public boolean hitLimitSwitch() {
        return !limitSwitch.get();
    }
}
