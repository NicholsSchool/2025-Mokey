package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.ElevatorConstants;
import org.firstinspires.ftc.teamcode.math_utils.AutoUtil;
import org.firstinspires.ftc.teamcode.math_utils.PIDController;

import java.util.function.IntSupplier;


/**
 * Robot Arm
 */
public class Elevator implements ElevatorConstants {
    private final DcMotorEx leftSlideMotor, rightSlideMotor;
    private final CRServoImplEx leftArm, rightArm, armGrabber;
    private final AnalogInput armEncoder;
    private final IntSupplier elevatorPosition;
    public final PIDController elevatorPID, armPID;
    private double elevatorSetpoint, armSetpoint;
    private ELEVATOR_STATE state;

    public enum ELEVATOR_STATE
    {
        MANUAL,
        GO_TO_POS,
        STOPPED
    }

    /**
     * Initializes the Arm
     *
     * @param hardwareMap the hardware map
     */
    public Elevator(HardwareMap hardwareMap, IntSupplier elevatorPosition) {
        leftSlideMotor = hardwareMap.get(DcMotorEx.class, "LeftClimberMotor");
        leftSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftSlideMotor.setDirection(DcMotorEx.Direction.REVERSE);

        rightSlideMotor = hardwareMap.get(DcMotorEx.class, "RightClimberMotor");
        rightSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setDirection(DcMotorEx.Direction.FORWARD);

        leftArm = hardwareMap.get(CRServoImplEx.class, "LeftArm");
        rightArm = hardwareMap.get(CRServoImplEx.class, "RightArm");

        armEncoder = hardwareMap.get(AnalogInput.class, "ArmEncoder");
        armGrabber = hardwareMap.get(CRServoImplEx.class, "ArmGrabber");

        elevatorSetpoint = 0.0;
        armSetpoint = 0.0;
        elevatorPID = new PIDController(SLIDE_P, 0.0, 0.0);
        armPID = new PIDController(0.01, 0.0, 0.0);
        this.elevatorPosition = elevatorPosition;
        this.state = ELEVATOR_STATE.GO_TO_POS;
    }

    public void periodic() {
        switch( state )
        {
            case MANUAL:
                this.elevatorSetpoint = this.getEncoderPosition();
                break;
            case GO_TO_POS:
                slideRawPower(-elevatorPID.calculate(this.getEncoderPosition(), this.elevatorSetpoint));
                break;
            case STOPPED:
            default:
                slideRawPower(0);
        }

        armRawPower(armPID.calculate(getArmPosition(), armSetpoint));

    }

    public int getEncoderPosition() { return elevatorPosition.getAsInt(); }

    public double getArmPosition() {
        return armEncoder.getVoltage() / 3.3 * 360.0;
    }

    public AutoUtil.AutoActionState setElevatorSetpoint(double setpoint) {
        this.state = ELEVATOR_STATE.GO_TO_POS;
        this.elevatorSetpoint = setpoint;
        if (Math.abs(setpoint - this.getEncoderPosition()) < 500) {
            return AutoUtil.AutoActionState.FINISHED;
        } else { return AutoUtil.AutoActionState.RUNNING; }
    }

    public void setArmSetpoint(double setpoint) {
        this.armSetpoint = setpoint;
    }

    public void manualControl(double power)
    {
        this.state = ELEVATOR_STATE.MANUAL;
        slideRawPower(power);
    }

    private void armRawPower(double power) {
        leftArm.setPower(power);
        rightArm.setPower(-power);
    }

    public void runArmGrabber(double power){
        armGrabber.setPower(power);
    }

    private void slideRawPower(double power){
        leftSlideMotor.setPower(power);
        rightSlideMotor.setPower(power);
    }

    public void stopElevator()
    {
        state = ELEVATOR_STATE.STOPPED;
    }

    public ELEVATOR_STATE getState()
    {
        return state;
    }

    public void setState(ELEVATOR_STATE state)
    {
        this.state = state;
    }
}