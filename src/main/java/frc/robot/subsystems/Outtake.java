package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.Ports.OuttakePorts;
import edu.wpi.first.wpilibj.DigitalInput;

public class Outtake extends SubsystemBase {
    
    private final SparkMax outtakeMotor;
    private final DigitalInput frontBB;
    private final DigitalInput middleBB;

    public Outtake() {
        outtakeMotor = new SparkMax(OuttakePorts.OUTTAKE_MOTOR, MotorType.kBrushless);

        frontBB = new DigitalInput(OuttakePorts.FRONT_BB);
        middleBB = new DigitalInput(OuttakePorts.MIDDLE_BB);

    }

    public Command runOuttakeMotorCmd() {
        return this.run(() -> outtakeMotor.set(OuttakeConstants.OUTTAKE_MOTOR_SPEED));
    }

    public Command reverseOuttakeMotorCmd() {
        return this.run(() -> outtakeMotor.set(-OuttakeConstants.OUTTAKE_MOTOR_SPEED));
    }

    public Command stopOuttakeMotorCmd() {
        return this.run(() -> outtakeMotor.set(0));
    }

    public boolean brokenFrontBB() {
        return !frontBB.get();
    }

    public boolean brokenMiddleBB() {
        return !middleBB.get();
    }

    public boolean coralInPosition() {
        return brokenFrontBB() && !brokenMiddleBB();
    }
}
