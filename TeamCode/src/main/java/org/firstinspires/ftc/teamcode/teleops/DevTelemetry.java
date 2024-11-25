package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.IndicatorConstants;
import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.math_utils.Vector;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.components.IndicatorLight;

import com.acmerobotics.dashboard.FtcDashboard;

@TeleOp(name="Dev Telemetry", group="Dev")
public class DevTelemetry extends OpMode {

    Drivetrain drivetrain;
    Elevator elevator;
    Intake intake;

    FtcDashboard dashboard;

    boolean showDrivetrainTelem = false;
    boolean showElevatorTelem = false;
    boolean showIntakeTelem = false;
    boolean showOtherTelem = false;

    Controller controller1;
    Controller controller2;

    @Override
    public void init() {

        drivetrain = new Drivetrain(hardwareMap, 0, 0, 0, false);
        elevator = new Elevator(hardwareMap);
        intake = new Intake(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

    }

    public void loop() {

        controller1.update();
        controller2.update();
        intake.periodic();
        elevator.periodic();

        drivetrain.leftLight.setColour(IndicatorLight.Colour.ORANGE);
        drivetrain.rightLight.setColour(IndicatorLight.Colour.ORANGE);

        telemetry.addLine(
                "Circle for Drivetrain, Triangle for Elevator, Square for Intake, X for Other"
        );
        telemetry.addData("Showing: ", (showDrivetrainTelem ? "Drivetrain" : "") +
                (showElevatorTelem ? " Elevator" : "") + (showIntakeTelem ? " Intake" : "") +
                (showOtherTelem ? " Other" : ""));

        if (controller1.circle.wasJustPressed()) { showDrivetrainTelem = !showDrivetrainTelem; }
        if (controller1.triangle.wasJustPressed()) { showElevatorTelem = !showElevatorTelem; }
        if (controller1.square.wasJustPressed()) { showIntakeTelem = !showIntakeTelem; }
        if (controller1.x.wasJustPressed()) { showOtherTelem = !showOtherTelem; }

        elevator.slideRawPower(-controller2.leftStick.y.value());
        intake.slideRawPower(-controller2.rightStick.y.value());
        elevator.setCarriageServoPower(controller2.leftTrigger.value() - controller2.rightTrigger.value());
        intake.runIntake(controller1.leftTrigger.value() - controller1.rightTrigger.value());
        drivetrain.drive(new Vector(controller1.leftStick.x.value(),controller1.leftStick.y.value()), controller1.rightStick.x.value(), true, true);

        if (showDrivetrainTelem) {
            telemetry.addLine("==========DRIVETRAIN==========");

            telemetry.addData("NavX Info", drivetrain.getNavxInfo());
            telemetry.addData("NavX Yaw", drivetrain.getPose().angle);
            telemetry.addData("x", drivetrain.getPose().x);
            telemetry.addData("y", drivetrain.getPose().y);

            telemetry.addLine("==============================\n");
        }

        if (showElevatorTelem) {
            telemetry.addLine("===========ELEVATOR===========");

            telemetry.addData("Encoder Position", elevator.getEncoderTicks());

            telemetry.addLine("==============================\n");
        }

        if (showIntakeTelem) {
            telemetry.addLine("=============INTAKE===========");

            telemetry.addData("Slide Encoder Position", intake.getEncoderTicks());
            telemetry.addData("Wrist Servo Positions", intake.getWristServoPositions());
            telemetry.addData("Slide Encoder Num Rotations", intake.slideEncoder.getNumRotations() );
            telemetry.addData("Slide Encoder Raw Velocity", intake.slideEncoder.getRawVelocity() );
            telemetry.addData("Slide Encoder Raw Position", intake.slideEncoder.getRawPosition() );

            telemetry.addLine("==============================\n");
        }


    }
}
