package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.IndicatorConstants;
import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import com.acmerobotics.dashboard.FtcDashboard;

import java.util.ResourceBundle;

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

    @Override
    public void init() {

        drivetrain = new Drivetrain(hardwareMap, 0, 0, 0, false);
        elevator = new Elevator(hardwareMap);
        intake = new Intake(hardwareMap, false);

        dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        controller1 = new Controller(gamepad1);

    }

    public void loop() {

        controller1.update();

        drivetrain.leftLight.setColourSequence(IndicatorConstants.ORANGE_FLASH, 1000);
        drivetrain.rightLight.setColourSequence(IndicatorConstants.ORANGE_FLASH, 1000);

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

        if (showDrivetrainTelem) {
            telemetry.addLine("==========DRIVETRAIN==========");

            telemetry.addData("Motor Velocities", drivetrain.getMotorVelocities());
            telemetry.addData("NavX Info", drivetrain.getNavxInfo());
            telemetry.addData("NavX Yaw", drivetrain.getYaw());
            telemetry.addData("Position", drivetrain.getPos());

            telemetry.addLine("==============================\n");
        }

        if (showElevatorTelem) {
            telemetry.addLine("===========ELEVATOR===========");

            telemetry.addData("Encoder Position", elevator.getEncoderTicks());

            telemetry.addLine("==============================\n");
        }

        if (showIntakeTelem) {
            telemetry.addLine("=============INTAKE===========");

            telemetry.addData("Slide Encoder Position", intake.getSlideTicks());
            telemetry.addData("Wrist Servo Positions", intake.getWristServoPositions());

            telemetry.addLine("==============================\n");
        }


    }

}
