package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="liteTesting", group="Dev")
public class liteTesting extends OpMode {

    private DcMotor leftClimbMotor, rightClimbMotor, slide;

    private AnalogInput armEncoder;


    @Override
    public void init() {
        leftClimbMotor = hardwareMap.get(DcMotor.class, "LeftClimberMotor");
        rightClimbMotor = hardwareMap.get(DcMotor.class, "RightClimberMotor");

        leftClimbMotor.setDirection(DcMotor.Direction.REVERSE);
        leftClimbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightClimbMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftClimbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        slide.setDirection(DcMotorEx.Direction.REVERSE);
        slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        armEncoder = hardwareMap.get(AnalogInput.class, "IntakeColor");
    }

    @Override
    public void loop() {
//        leftClimbMotor.setPower(gamepad1.left_stick_y);
//        rightClimbMotor.setPower(gamepad1.left_stick_y);
//        slide.setPower(gamepad1.right_stick_y);

        telemetry.addData("arm", armEncoder.getVoltage() / 3.3 * 360);
    }

}
