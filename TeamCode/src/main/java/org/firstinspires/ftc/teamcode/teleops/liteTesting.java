package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="liteTesting", group="Dev")
public class liteTesting extends OpMode {

    private DcMotor leftClimbMotor, rightClimbMotor, slide;
    private CRServoImplEx leftArm, rightArm, intakeWrist;
    private AnalogInput armEncoder, wristEncoder;


    @Override
    public void init() {
        leftClimbMotor = hardwareMap.get(DcMotor.class, "LeftClimberMotor");
        rightClimbMotor = hardwareMap.get(DcMotor.class, "RightClimberMotor");

        leftClimbMotor.setDirection(DcMotor.Direction.FORWARD);
        leftClimbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightClimbMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftClimbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightClimbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightClimbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slide = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setDirection(DcMotorEx.Direction.REVERSE);
        slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        leftArm = hardwareMap.get(CRServoImplEx.class, "LeftArm");
        rightArm = hardwareMap.get(CRServoImplEx.class, "RightArm");

        armEncoder = hardwareMap.get(AnalogInput.class, "ArmEncoder");

        intakeWrist = hardwareMap.get(CRServoImplEx.class, "WristServo");
        wristEncoder = hardwareMap.get(AnalogInput.class, "WristEncoder");
    }

    @Override
    public void loop() {

        leftArm.setPower(gamepad1.left_stick_y);
        rightArm.setPower(-gamepad1.left_stick_y);

        telemetry.addData("Elevator Real", rightClimbMotor.getCurrentPosition());
        telemetry.addData("Intake Real", slide.getCurrentPosition());
        telemetry.addData("Wrist Real", (wristEncoder.getVoltage() / 3.3 * 360.0));
        telemetry.addData("Arm Real", (armEncoder.getVoltage() / 3.3 * 360.0));
        telemetry.update();

    }
}
