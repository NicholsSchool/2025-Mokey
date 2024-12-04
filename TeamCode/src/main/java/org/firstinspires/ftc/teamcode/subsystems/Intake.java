package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.math_utils.PIDController;
//import org.firstinspires.ftc.teamcode.subsystems.components.OctoEncoder;

public class Intake implements IntakeConstants {
    private final DcMotorEx slide;
//    private final OctoEncoder slideEncoder;
    private final CRServoImplEx intakeWristF, intakeWristB;
    private final CRServoImplEx intakeOne,intakeTwo;
    private final AnalogInput wristFEncoder, wristBEncoder;

    private double intakeSetpoint;
    private double wristSetpoint;
    private final PIDController slidePid;
    private final PIDController wristFPid;
    private final PIDController wristBPid;
    //public final ColorSensor colorSensor;

    public Intake(HardwareMap hwMap) {
        slide = hwMap.get(DcMotorEx.class, "IntakeMotor");
        slide.setDirection(DcMotorEx.Direction.REVERSE);
        slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

//        slideEncoder = new OctoEncoder(hwMap, SLIDE_ENC_ID, OctoQuadBase.EncoderDirection.REVERSE);
//        slideEncoder.reset();

        intakeOne = hwMap.get(CRServoImplEx.class, "IntakeLeft");
        intakeTwo = hwMap.get(CRServoImplEx.class, "IntakeRight");

        intakeOne.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeTwo.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeWristF = hwMap.get(CRServoImplEx.class, "WristFront");
        intakeWristB = hwMap.get(CRServoImplEx.class, "WristBack");

        intakeWristF.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeWristB.setDirection(DcMotorSimple.Direction.FORWARD);

        wristFEncoder = hwMap.get(AnalogInput.class, "WristFrontEncoder");
        wristBEncoder = hwMap.get(AnalogInput.class, "WristBackEncoder");

        wristSetpoint = INTAKE_WRIST_FRONT_IN;
        wristFPid = new PIDController( 0.005, 0.0, 0.0 );
        wristBPid = new PIDController( 0.005, 0.0,0.0);
// 88,220
//186, 318
        intakeSetpoint = 0.0;
        slidePid = new PIDController( INTAKE_P, 0.0, 0.0 );


        //colorSensor = hwMap.get(ColorSensor.class, "IntakeColor");
    }

    public void periodic() {
//        if (Math.abs(getEncoderPosition() - intakeSetpoint) > 1000) {
//            this.slideRawPower(-slidePid.calculate(slideEncoder.getPosition(), intakeSetpoint));
//        }

      intakeWristF.setPower( -wristFPid.calculate( this.getWristServoPositions()[0], ( wristSetpoint ) ) );
      intakeWristB.setPower( -wristBPid.calculate( this.getWristServoPositions()[1], wristSetpoint - 65) );
    }

//    public int getEncoderPosition() { return slideEncoder.getPosition(); }
//
//    public int getEncoderVelocity() { return slideEncoder.getVelocity(); }

    public double[] getWristServoPositions() {
        return new double[]{ wristFEncoder.getVoltage() / 3.3 * 360.0, wristBEncoder.getVoltage() / 3.3 * 360.0} ;
    }
    public void slideRawPower(double power){
        slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slide.setPower(power * SLIDE_SPEED);
    }

    public void setIntakeSetpoint(double intakeSetpoint){
        this.intakeSetpoint = intakeSetpoint;
    }

    public void setPIDCoefficients(double kP, double kI, double kD) {
        slidePid.setPID(kP, kI, kD);
    }

    public void runIntake(double power){
        intakeOne.setPower(power * INTAKE_SPEED);
        intakeTwo.setPower(power * INTAKE_SPEED);
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

}
