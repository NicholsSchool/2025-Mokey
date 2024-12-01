package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuadBase;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.ElevatorConstants;
import org.firstinspires.ftc.teamcode.math_utils.PIDController;
import org.firstinspires.ftc.teamcode.subsystems.components.OctoEncoder;


/**
 * Robot Arm
 */
public class Elevator implements ElevatorConstants {
    private final DcMotorEx leftSlideMotor, rightSlideMotor;
    private final OctoEncoder slideEncoder;
    public final PIDController pidController;
    private double setpoint;

    /**
     * Initializes the Arm
     *
     * @param hardwareMap the hardware map
     */
    public Elevator(HardwareMap hardwareMap) {
        leftSlideMotor = hardwareMap.get(DcMotorEx.class, "LeftClimberMotor");
        leftSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftSlideMotor.setDirection(DcMotorEx.Direction.REVERSE);

        rightSlideMotor = hardwareMap.get(DcMotorEx.class, "RightClimberMotor");
        rightSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setDirection(DcMotorEx.Direction.FORWARD);

        slideEncoder = new OctoEncoder(hardwareMap, SLIDE_ENC_ID, OctoQuadBase.EncoderDirection.FORWARD);
        slideEncoder.reset();

        setpoint = 0.0;
        pidController = new PIDController(SLIDE_P, 0.0, 0.0);
    }

    public void periodic() {
        slideRawPower( -pidController.calculate( getEncoderPosition(), setpoint ));
    }

    public int getEncoderPosition() { return slideEncoder.getPosition(); }

    public int getEncoderVelocity() { return slideEncoder.getVelocity(); }

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