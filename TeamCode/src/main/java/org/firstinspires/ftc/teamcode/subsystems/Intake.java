package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuadBase;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.math_utils.PIDController;
import org.firstinspires.ftc.teamcode.subsystems.components.OctoEncoder;

public class Intake implements IntakeConstants {
    private final DcMotorEx slide;
    private final OctoEncoder slideEncoder;
    private final ServoImplEx intakeWristF, intakeWristB;
    private final CRServoImplEx intakeOne,intakeTwo;

    private double setpoint;
    private final PIDController pidController;
    //public final ColorSensor colorSensor;

    public Intake(HardwareMap hwMap) {
        slide = hwMap.get(DcMotorEx.class, "IntakeMotor");
        slide.setDirection(DcMotorEx.Direction.REVERSE);
        slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        slideEncoder = new OctoEncoder(hwMap, SLIDE_ENC_ID, OctoQuadBase.EncoderDirection.REVERSE);
        slideEncoder.reset();

        intakeOne = hwMap.get(CRServoImplEx.class, "IntakeLeft");
        intakeTwo = hwMap.get(CRServoImplEx.class, "IntakeRight");

        intakeOne.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeTwo.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeWristF = hwMap.get(ServoImplEx.class, "WristFront");
        intakeWristB = hwMap.get(ServoImplEx.class, "WristBack");

        intakeWristF.setDirection(Servo.Direction.FORWARD);
        intakeWristB.setDirection(Servo.Direction.FORWARD);

        setpoint = 0.0;
        pidController = new PIDController( INTAKE_P, 0.0, 0.0 );


        //colorSensor = hwMap.get(ColorSensor.class, "IntakeColor");
    }

    public void periodic() {
        if (Math.abs(getEncoderPosition() - setpoint) > 1000) {
            this.slideRawPower(-pidController.calculate(slideEncoder.getPosition(), setpoint));
        }
    }

    public int getEncoderPosition() { return slideEncoder.getPosition(); }

    public int getEncoderVelocity() { return slideEncoder.getVelocity(); }

    public double[] getWristServoPositions() {
        return new double[]{intakeWristF.getPosition(), intakeWristB.getPosition()};
    }

    public void slideRawPower(double power){
        slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slide.setPower(power * SLIDE_SPEED);
    }

    public void setSetpoint(double setpoint){
        this.setpoint = setpoint;
    }

    public void setPIDCoefficients(double kP, double kI, double kD) {
        pidController.setPID(kP, kI, kD);
    }

    public void runIntake(double power){
        intakeOne.setPower(power * INTAKE_SPEED);
        intakeTwo.setPower(power * INTAKE_SPEED);
    }

    public void wristToPos(double pos){
        intakeWristF.setPosition(pos);
        intakeWristB.setPosition(pos);
    }

}
