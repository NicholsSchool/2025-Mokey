package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.ElevatorConstants;
import org.firstinspires.ftc.teamcode.math_utils.AutoUtil;
import org.firstinspires.ftc.teamcode.math_utils.PIDController;


/**
 * Robot Arm
 */
public class Elevator implements ElevatorConstants {
    private final DcMotorEx leftSlideMotor, rightSlideMotor;
    private final CRServoImplEx leftArm, rightArm, armGrabber;
    private final AnalogInput armEncoder;
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
     * @param hardwareMap          the hardware map
     * @param suppressEncoderReset set to false to reset encoders
     */
    public Elevator(HardwareMap hardwareMap, boolean suppressEncoderReset) {
        leftSlideMotor = hardwareMap.get(DcMotorEx.class, "LeftClimberMotor");
        leftSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftSlideMotor.setDirection(DcMotorEx.Direction.FORWARD);

        rightSlideMotor = hardwareMap.get(DcMotorEx.class, "RightClimberMotor");
        rightSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setDirection(DcMotorEx.Direction.FORWARD);

        leftArm = hardwareMap.get(CRServoImplEx.class, "LeftArm");
        rightArm = hardwareMap.get(CRServoImplEx.class, "RightArm");

        armEncoder = hardwareMap.get(AnalogInput.class, "ArmEncoder");
        armGrabber = hardwareMap.get(CRServoImplEx.class, "ArmGrabber");

        if (!suppressEncoderReset) resetSlideEncoder();

        elevatorSetpoint = 0.0;
        armSetpoint = ARM_STOW;
        elevatorPID = new PIDController(SLIDE_P, SLIDE_I, SLIDE_D);
        armPID = new PIDController(ARM_P, 0.0, 0.0);
        this.state = ELEVATOR_STATE.GO_TO_POS;
    }

    public void periodic() {
        switch( state )
        {
            case MANUAL:
                this.elevatorSetpoint = this.getEncoderPosition();
                break;
            case GO_TO_POS:
                slideRawPower(elevatorPID.calculate(this.getEncoderPosition(), this.elevatorSetpoint));
                break;
            case STOPPED:
            default:
                slideRawPower(0);
        }

        armRawPower(armPID.calculate(getArmPosition(), armSetpoint));

    }

    public int getEncoderPosition() { return rightSlideMotor.getCurrentPosition(); }

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

    public AutoUtil.AutoActionState setArmSetpoint(double setpoint) {

        if (Math.abs(getArmPosition() - setpoint) < 5) return AutoUtil.AutoActionState.FINISHED;

        this.armSetpoint = setpoint;

        return AutoUtil.AutoActionState.RUNNING;
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
        leftSlideMotor.setPower(-power);
        rightSlideMotor.setPower(power);
    }

    public void resetSlideEncoder() {
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    public String getTelemetry() {
        StringBuilder telemBuilder = new StringBuilder();
        String lineSep = System.lineSeparator();

        telemBuilder.append("Elevator Desired: ").append(elevatorSetpoint).append(lineSep);
        telemBuilder.append("Elevator Real: ").append(getEncoderPosition()).append(lineSep);
        telemBuilder.append("Elevator Power: ").append(leftSlideMotor.getPower()).append(lineSep);
        telemBuilder.append("Elevator State: ").append(state).append(lineSep);
        telemBuilder.append("Arm Desired: ").append(armSetpoint).append(lineSep);
        telemBuilder.append("Arm Pos: ").append(getArmPosition()).append(lineSep);
        telemBuilder.append("Arm Power: ").append(leftArm.getPower()).append(lineSep);

        return telemBuilder.toString();
    }
}