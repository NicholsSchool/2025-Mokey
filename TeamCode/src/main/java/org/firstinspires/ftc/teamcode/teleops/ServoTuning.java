package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name = "Servo Tuning", group = "Dev")
public class ServoTuning extends OpMode {

    Intake intake;
    Elevator elevator;

    @Override
    public void init() {

        intake = new Intake(hardwareMap, false);
        elevator = new Elevator(hardwareMap, false);

    }

    @Override
    public void loop() {

        intake.setWristSetpoint(180.0);
        elevator.setArmSetpoint(180.0);

        intake.periodic();
        elevator.periodic();

    }
}
