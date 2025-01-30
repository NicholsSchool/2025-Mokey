package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.constants.ElevatorConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.components.IndicatorLight;

import com.acmerobotics.dashboard.FtcDashboard;

import java.util.Arrays;

@TeleOp(name="[COMP] TeleOp", group="Dev")
public class CompTeleOp extends OpMode {

    Drivetrain drivetrain;
    Elevator elevator;
    Intake intake;

    Controller controller1;
    Controller controller2;

    @Override
    public void init() {
        Pose2D initPose = new Pose2D(DistanceUnit.INCH, -24, 54, AngleUnit.DEGREES, 0);
        drivetrain = new Drivetrain(hardwareMap, initPose, 270, false);
        elevator = new Elevator(hardwareMap, drivetrain::getElevatorPosition);
        drivetrain.resetElevatorEncoder();
        intake = new Intake(hardwareMap);

        telemetry.setMsTransmissionInterval(50);

        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

    }

    public void loop() {

        controller1.update();
        controller2.update();
        drivetrain.update();

        //drivetrain.leftLight.setColour(IndicatorLight.Colour.ORANGE);
        //drivetrain.rightLight.setColour(IndicatorLight.Colour.ORANGE);

        if (controller1.share.wasJustPressed()) drivetrain.resetIMU();

        //AUTO ALIGN
        if (controller1.circle.wasJustPressed()) drivetrain.setTargetHeading(0);
        if (controller1.square.wasJustPressed()) drivetrain.setTargetHeading(Math.PI);
        if (controller1.triangle.wasJustPressed()) drivetrain.setTargetHeading(Math.PI / 2);
        if (controller1.x.wasJustPressed()) drivetrain.setTargetHeading(3 * Math.PI / 2);

        //ELEVATOR MANUAL
        if( Math.abs(controller2.leftStick.y.value()) > 0.05 )
            elevator.manualControl( -controller2.leftStick.y.value() );
        else elevator.setState(Elevator.ELEVATOR_STATE.GO_TO_POS);

        //ELEVATOR SETPOINTS
        if (controller2.x.wasJustPressed()) elevator.setSetpoint(ElevatorConstants.WAYPOINT_ZERO);

        if (controller2.rightBumper.wasJustPressed()) elevator.setSetpoint(ElevatorConstants.SPECIMEN_READY);
        if (controller2.rightBumper.wasJustReleased()) elevator.setSetpoint(ElevatorConstants.SPECIMEN_PULL);

        if (controller2.leftBumper.wasJustPressed()) elevator.setSetpoint(ElevatorConstants.CLIMB_READY);
        if (controller2.leftBumper.wasJustReleased()) elevator.setSetpoint(ElevatorConstants.CLIMB_PULL);

        elevator.periodic();

        //INTAKE MANUAL
        if ( Math.abs(controller2.rightStick.y.value()) > 0.05 ) {
            intake.manualControl(-controller2.rightStick.toVector().y);
        } else {
            intake.setIntakeState(Intake.INTAKE_STATE.GO_TO_POS);
        }

        //INTAKE SETPOINTS
        if( controller2.dpadUp.wasJustPressed() )
            intake.setIntakeSetpoint(IntakeConstants.WAYPOINT_EXTEND);
        if( controller2.dpadDown.wasJustPressed())
            intake.setIntakeSetpoint(IntakeConstants.WAYPOINT_RETRACT);

        //WRIST SETPOINTS
        intake.setWristSetpoint(controller2.triangle.isPressed() ? Intake.WristState.IN: Intake.WristState.OUT );

        //INTAKE WHEEL
        intake.runIntake(controller1.leftTrigger.value() - controller1.rightTrigger.value());

        intake.periodic();

        drivetrain.drive(controller1.leftStick.toVector(), controller1.rightStick.toVector().x, false);

        telemetry.addData("Robot Pose", drivetrain.getPose().toString());
        telemetry.addData("Elevator Desired Position", elevator.pidController.getSetpoint());
        telemetry.addData("Elevator Position", elevator.getEncoderPosition());
        telemetry.addData("Intake Desired Position", elevator.pidController.getSetpoint());
        telemetry.addData("Intake Position", intake.getIntakeSlidePos());
        telemetry.addData("Wrist Servo Positions", Arrays.toString(intake.getWristServoPositions()));
    }
}
