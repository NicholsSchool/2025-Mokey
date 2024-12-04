package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.ElevatorConstants;
import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.components.IndicatorLight;

import com.acmerobotics.dashboard.FtcDashboard;

import java.util.Arrays;

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

        drivetrain = new Drivetrain(hardwareMap, 0, 0, Math.PI, false);
        elevator = new Elevator(hardwareMap, drivetrain::getElevatorPosition);
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
        drivetrain.update();

        drivetrain.leftLight.setColour(IndicatorLight.Colour.ORANGE);
        drivetrain.rightLight.setColour(IndicatorLight.Colour.ORANGE);

        telemetry.addLine(
                "Circle for Drivetrain, Triangle for Elevator, Square for Intake, X for Other"
        );
        telemetry.addData("Showing: ", (showDrivetrainTelem ? "Drivetrain" : "") +
                (showElevatorTelem ? " Elevator" : "") + (showIntakeTelem ? " Intake" : "") +
                (showOtherTelem ? " Other" : ""));

        if (controller1.options.wasJustPressed()) drivetrain.resetIMU();

//        if (controller1.circle.wasJustPressed()) drivetrain.setTargetHeading(0);
//        if (controller1.square.wasJustPressed()) drivetrain.setTargetHeading(Math.PI);
//        if (controller1.triangle.wasJustPressed()) drivetrain.setTargetHeading(Math.PI / 2);
//        if (controller1.x.wasJustPressed()) drivetrain.setTargetHeading(3 * Math.PI / 2);


        if (controller1.x.wasJustPressed()) {
            elevator.setSetpoint(ElevatorConstants.WAYPOINT_ZERO);
        }

        if (controller1.rightBumper.wasJustPressed()) elevator.setSetpoint(ElevatorConstants.WAYPOINT_CHAMBER_READY);
        if (controller1.rightBumper.wasJustReleased()) elevator.setSetpoint(ElevatorConstants.WAYPOINT_CHAMBER_PULL);

//        if (controller1.leftBumper.wasJustPressed()) elevator.setSetpoint(ElevatorConstants.WAYPOINT_CLIMB_READY);
//        if (controller1.leftBumper.wasJustReleased()) {
//            elevator.setSetpoint(ElevatorConstants.WAYPOINT_CLIMB_PULL);
//        }
        // else elevator.setPIDCoefficients(ElevatorConstants.SLIDE_P, 0.0, 0.0);

        elevator.periodic();

        intake.setWristSetpoint(controller1.leftBumper.isPressed() ? Intake.WristState.IN: Intake.WristState.OUT );
        intake.slideRawPower(controller1.rightStick.toVector().y);
        intake.runIntake(controller1.leftTrigger.value() - controller1.rightTrigger.value());

        intake.periodic();

        drivetrain.drive(controller1.leftStick.toVector(), controller1.rightStick.toVector().x, false);

            telemetry.addData("Robot Pose", drivetrain.getPose().toString());
            //telemetry.addData("Drive Motor Velocities", Arrays.toString(drivetrain.getMotorVelocities()));

            telemetry.addData("Elevator Desired Position", elevator.pidController.getSetpoint());
            telemetry.addData("Elevator Encoder Position", elevator.getEncoderPosition());

            telemetry.addData("Wrist Servo Positions", Arrays.toString(intake.getWristServoPositions()));
//            telemetry.addData("Intake Encoder Velocity", intake.getEncoderVelocity() );
//            telemetry.addData("Intake Encoder Position", intake.getEncoderPosition() );
    }
}
