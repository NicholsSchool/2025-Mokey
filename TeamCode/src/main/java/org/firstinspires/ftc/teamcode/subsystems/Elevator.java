package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuadBase;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.ElevatorConstants;
import org.firstinspires.ftc.teamcode.math_utils.PIDController;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;


/**
 * Robot Arm
 */
public class Elevator implements ElevatorConstants {
    private final DcMotorEx leftSlideMotor, rightSlideMotor;
    private final IntSupplier elevatorPosition;
    public final PIDController pidController;
    private double setpoint;

    /**
     * Initializes the Arm
     *
     * @param hardwareMap the hardware map
     */
    public Elevator(HardwareMap hardwareMap, IntSupplier elevatorPosition ) {
        leftSlideMotor = hardwareMap.get(DcMotorEx.class, "LeftClimberMotor");
        leftSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftSlideMotor.setDirection(DcMotorEx.Direction.REVERSE);

        rightSlideMotor = hardwareMap.get(DcMotorEx.class, "RightClimberMotor");
        rightSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setDirection(DcMotorEx.Direction.FORWARD);


        setpoint = 0.0;
        pidController = new PIDController(SLIDE_P, 0.0, 0.0);
        this.elevatorPosition = elevatorPosition;
    }

    public void periodic() {
        slideRawPower( -pidController.calculate( this.getEncoderPosition(), setpoint ));
    }

    public int getEncoderPosition() { return elevatorPosition.getAsInt(); }


    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void setPIDCoefficients(double kP, double kI, double kD) {
        pidController.setPID(kP, kI, kD);
    }

    public void slideRawPower(double power){
        leftSlideMotor.setPower(power);
        rightSlideMotor.setPower(power);
    }
}