package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.math_utils.PIDController;
//import org.firstinspires.ftc.teamcode.subsystems.components.OctoEncoder;

public class Intake implements IntakeConstants {
    private final DcMotorEx slide;
//    private final OctoEncoder slideEncoder;
    private final CRServoImplEx intakeWristF, intakeWristB;
    private final CRServoImplEx intakeCR;
    private final AnalogInput wristFEncoder, wristBEncoder;

    private double intakeSetpoint;
    private double wristSetpoint;
    private final PIDController slidePid;
    private final PIDController wristFPid;
    private final PIDController wristBPid;
    private final DigitalChannelImpl intakeZero;

    private INTAKE_STATE intakeState;

    public enum INTAKE_STATE
    {
        MANUAL,
        GO_TO_POS,
        STOPPED
    }

    public Intake(HardwareMap hwMap) {
        slide = hwMap.get(DcMotorEx.class, "IntakeMotor");
        slide.setDirection(DcMotorEx.Direction.FORWARD);
        slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeCR = hwMap.get(CRServoImplEx.class, "IntakeServo");

        intakeCR.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeWristF = hwMap.get(CRServoImplEx.class, "WristFront");
        intakeWristB = hwMap.get(CRServoImplEx.class, "WristBack");

        intakeWristF.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeWristB.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeZero = hwMap.get(DigitalChannelImpl.class, "IntakeMagnet");

        wristFEncoder = hwMap.get(AnalogInput.class, "WristFrontEncoder");
        wristBEncoder = hwMap.get(AnalogInput.class, "WristBackEncoder");

        wristSetpoint = INTAKE_WRIST_FRONT_IN;
        wristFPid = new PIDController( 0.005, 0.0, 0.0 );
        wristBPid = new PIDController( 0.005, 0.0,0.0);

        intakeSetpoint = 0.0;
        slidePid = new PIDController(SLIDE_P, 0.0, 0.0 );

        intakeState = INTAKE_STATE.STOPPED;
    }

    public void periodic() {

        switch  (intakeState) {
            case MANUAL:
                setIntakeSetpoint(getIntakeSlidePos());
                break;
            case GO_TO_POS:
                slideRawPower(slidePid.calculate(getIntakeSlidePos(), intakeSetpoint));
                break;
            case STOPPED:
            default:
                slideRawPower(0);
        }

        if( intakeZero.getState() ) this.resetSlideEncoder();


        intakeWristF.setPower( -wristFPid.calculate( this.getWristServoPositions()[0], ( wristSetpoint ) ) );
        intakeWristB.setPower( -wristBPid.calculate( this.getWristServoPositions()[1], wristSetpoint - 75) );
    }

    public int getIntakeSlidePos() { return slide.getCurrentPosition(); }

    public double getEncoderVelocity() { return slide.getVelocity(); }

    public double[] getWristServoPositions() {
        return new double[]{ wristFEncoder.getVoltage() / 3.3 * 360.0, wristBEncoder.getVoltage() / 3.3 * 360.0} ;
    }
    private void slideRawPower(double power){
        slide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slide.setPower(power);
    }

    public void manualControl(double power){
       this.intakeState = INTAKE_STATE.MANUAL;
       slideRawPower(power);
    }

    public void resetSlideEncoder(){
        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setIntakeSetpoint(double intakeSetpoint){
        this.intakeState = INTAKE_STATE.GO_TO_POS;
        this.intakeSetpoint = intakeSetpoint;
    }

    public void setIntakeState(INTAKE_STATE state) {
        this.intakeState = state;
    }

    public void runIntake(double power){
        intakeCR.setPower(power);
        //intakeTwo.setPower(power * INTAKE_SPEED);
    }

    public INTAKE_STATE getSlideState() {
        return intakeState;
    }

    public void setWristSetpoint(WristState wristState){
        switch( wristState )
        {
            case IN:
                wristSetpoint = INTAKE_WRIST_FRONT_IN;
                break;
            case OUT:
                wristSetpoint = INTAKE_WRIST_FRONT_OUT;
                break;
        }
    }

    public enum WristState {
        IN,
        OUT
    }

    public String getTelemetry() {
        StringBuilder telemBuilder = new StringBuilder();
        String lineSep = System.lineSeparator();

        telemBuilder.append("Intake Slide Setpoint: ").append(intakeSetpoint).append(lineSep);
        telemBuilder.append("Intake Slide Real").append(getIntakeSlidePos()).append(lineSep);
        telemBuilder.append("Wrist Desired Pos").append(wristSetpoint).append(lineSep);
        telemBuilder.append("Wrist Real").append(getWristServoPositions()[0]).append(lineSep);

        return telemBuilder.toString();
    }

}
