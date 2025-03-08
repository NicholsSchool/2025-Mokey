package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.math_utils.AutoUtil;
import org.firstinspires.ftc.teamcode.math_utils.PIDController;
//import org.firstinspires.ftc.teamcode.subsystems.components.OctoEncoder;

public class Intake implements IntakeConstants {
    private final DcMotorEx slide;
    private final CRServoImplEx intakeWristF;
    private final CRServoImplEx intakeCR;
    private final AnalogInput wristEncoder;

    private double intakeSetpoint;
    private double wristSetpoint;
    private final PIDController slidePID;
    private final PIDController wristPID;
    private final DigitalChannelImpl intakeZero;

    private INTAKE_STATE intakeState;
    private WRIST_STATE wristState;

    public enum INTAKE_STATE {
        MANUAL,
        GO_TO_POS,
        STOPPED
    }

    public enum WRIST_STATE {
        MANUAL,
        GO_TO_POS
    }

    public enum WRIST_SETPOINT {
        UP,
        DOWN,
        STOW
    }

    public Intake(HardwareMap hwMap, boolean suppressEncoderReset) {
        slide = hwMap.get(DcMotorEx.class, "IntakeMotor");
        slide.setDirection(DcMotorEx.Direction.FORWARD);
        slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeCR = hwMap.get(CRServoImplEx.class, "IntakeServo");

        intakeCR.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeWristF = hwMap.get(CRServoImplEx.class, "WristServo");

        intakeWristF.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeZero = hwMap.get(DigitalChannelImpl.class, "IntakeMagnet");

        wristEncoder = hwMap.get(AnalogInput.class, "WristEncoder");

        wristSetpoint = WRIST_HANDOFF;
        wristState = WRIST_STATE.GO_TO_POS;
        wristPID = new PIDController( WRIST_P, 0.0, 0.0 );

        intakeSetpoint = 0.0;
        slidePID = new PIDController(SLIDE_P, 0.0, 0.0 );

        if (!suppressEncoderReset) resetSlideEncoder();

        intakeState = INTAKE_STATE.STOPPED;
    }

    public void periodic() {

        if( !intakeZero.getState() ) this.resetSlideEncoder();

        switch  (intakeState) {
            case MANUAL:
                this.intakeSetpoint = getIntakeSlidePos();
                break;
            case GO_TO_POS:
                slideRawPower(slidePID.calculate(getIntakeSlidePos(), intakeSetpoint));
                break;
            case STOPPED:
            default:
                slideRawPower(0);
        }

        switch (wristState) {
            case MANUAL:
                this.wristSetpoint = getWristServoPosition();
                break;
            case GO_TO_POS:
                intakeWristF.setPower( -wristPID.calculate( this.getWristServoPosition(), ( wristSetpoint ) ) );
        }
    }

    public int getIntakeSlidePos() { return slide.getCurrentPosition(); }

    public double getWristServoPosition() {
        return wristEncoder.getVoltage() / 3.3 * 360.0;
    }

    private void slideRawPower(double power){
        slide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slide.setPower(power);
    }

    public void slideManual(double power){
       this.intakeState = INTAKE_STATE.MANUAL;
       //break for soft limit
       if ((!intakeZero.getState() || getIntakeSlidePos() < -1000) && power < 0) { return; }
       //half power throttle for overcurrent
       if( slide.getCurrent(CurrentUnit.AMPS) > SLIDE_CURRENT_LIMIT ) { slideRawPower(0.5 * power); return; }

       slideRawPower(power);
    }

    public void wristManual (double power) {
        this.wristState = WRIST_STATE.MANUAL;
        intakeWristF.setPower(power);
    }

    public void resetSlideEncoder(){
        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public AutoUtil.AutoActionState setIntakeSetpoint(double intakeSetpoint){
        this.intakeState = INTAKE_STATE.GO_TO_POS;
        if (Math.abs(intakeSetpoint - this.getIntakeSlidePos()) < 500) {
            return AutoUtil.AutoActionState.FINISHED;
        }
        this.intakeSetpoint = intakeSetpoint;
        return AutoUtil.AutoActionState.RUNNING;
    }

    public void setIntakeState(INTAKE_STATE state) {
        this.intakeState = state;
    }

    public void setWristState(WRIST_STATE state) {
        this.wristState = state;
    }

    public void runIntake(double power){
        intakeCR.setPower(power);
        //intakeTwo.setPower(power * INTAKE_SPEED);
    }

    public AutoUtil.AutoActionState setWristSetpoint(double wristSetpoint){

        if (Math.abs(getWristServoPosition() - wristSetpoint) < 5) return AutoUtil.AutoActionState.FINISHED;

        this.wristSetpoint = wristSetpoint;

        this.wristState = WRIST_STATE.GO_TO_POS;

        return AutoUtil.AutoActionState.RUNNING;
    }

    public String getTelemetry() {
        StringBuilder telemBuilder = new StringBuilder();
        String lineSep = System.lineSeparator();

        telemBuilder.append("Intake Slide Setpoint: ").append(intakeSetpoint).append(lineSep);
        telemBuilder.append("Intake Slide Real: ").append(getIntakeSlidePos()).append(lineSep);
        telemBuilder.append("Intake State: ").append(intakeState).append(lineSep);
        telemBuilder.append("Wrist Desired Pos: ").append(wristSetpoint).append(lineSep);
        telemBuilder.append("Wrist Real: ").append(getWristServoPosition()).append(lineSep);

        return telemBuilder.toString();
    }

}
