package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuadBase;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.ElevatorConstants;
import org.firstinspires.ftc.teamcode.subsystems.components.OctoEncoder;


/**
 * Robot Arm
 */
public class Elevator implements ElevatorConstants {
    private final DcMotorEx leftSlideMotor, rightSlideMotor;
    private final OctoEncoder slideEncoder;

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
    }

    public int getEncoderPosition() { return slideEncoder.getPosition(); }

    public int getEncoderVelocity() { return slideEncoder.getVelocity(); }

    public void setSlidePosition(double inputPosition) {
        //TODO: Use SimpleFeedbackController for this
    }

    public void slideRawPower(double power){
        leftSlideMotor.setPower(power);
        rightSlideMotor.setPower(power);
    }
}