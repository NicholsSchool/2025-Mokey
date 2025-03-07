package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

@TeleOp(name="[COMP] TeleOp", group="Dev")
public class CompTeleOp extends OpMode {

    Drivetrain drivetrain;
    Elevator elevator;
    Intake intake;
    FtcDashboard dashboard;

    Controller controller1;
    Controller controller2;

    @Override
    public void init() {
        Pose2D initPose = new Pose2D(DistanceUnit.INCH, -24, 54, AngleUnit.DEGREES, 0);
        drivetrain = new Drivetrain(hardwareMap, initPose, 270, false, true);
        elevator = new Elevator(hardwareMap, true);
        intake = new Intake(hardwareMap, true);

        intake.resetSlideEncoder();
        elevator.resetSlideEncoder();

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);


        drivetrain.leftLight.setColour(IndicatorLight.Colour.GREEN);
        drivetrain.rightLight.setColour(IndicatorLight.Colour.GREEN);
    }

    public void loop() {

        controller1.update();
        controller2.update();
        drivetrain.update();

        drivetrain.leftLight.setColour(IndicatorLight.Colour.YELLOW);
        drivetrain.rightLight.setColour(IndicatorLight.Colour.YELLOW);

        if (controller1.share.wasJustPressed()) drivetrain.resetIMU();

        //AUTO ALIGN
//        if (controller1.circle.wasJustPressed()) drivetrain.setTargetHeading(0);
//        if (controller1.square.wasJustPressed()) drivetrain.setTargetHeading(Math.PI);
//        if (controller1.triangle.wasJustPressed()) drivetrain.setTargetHeading(Math.PI / 2);
//        if (controller1.x.wasJustPressed()) drivetrain.setTargetHeading(3 * Math.PI / 2);

        //ELEVATOR MANUAL
        if( Math.abs(controller2.leftStick.y.value()) > 0.05 )
            elevator.manualControl( -controller2.leftStick.y.value() );
        else elevator.setState(Elevator.ELEVATOR_STATE.GO_TO_POS);

        //ELEVATOR SETPOINTS
        if (controller2.x.wasJustPressed()) elevator.setElevatorSetpoint(ElevatorConstants.WAYPOINT_ZERO);

        if (controller2.rightBumper.wasJustPressed()) elevator.setElevatorSetpoint(ElevatorConstants.SPECIMEN_READY);
        if (controller2.rightBumper.wasJustReleased()) elevator.setElevatorSetpoint(ElevatorConstants.SPECIMEN_PULL);

        elevator.periodic();

        //INTAKE MANUAL
        if ( Math.abs(controller2.rightStick.y.value()) > 0.05 ) {
            intake.slideManual(controller2.rightStick.toVector().y);
        } else {
            intake.setIntakeState(Intake.INTAKE_STATE.GO_TO_POS);
        }

        //WRIST MANUAL
        if( controller2.dpadUp.isPressed() )
            intake.wristManual(-1);
        else if( controller2.dpadDown.wasJustPressed())
            intake.wristManual(1);
        else intake.setWristState(Intake.WRIST_STATE.GO_TO_POS);

        //AUTO HANDOFF
        if (intake.getIntakeSlidePos() < IntakeConstants.WAYPOINT_STOW && !controller2.triangle.isPressed()) {
            intake.setWristSetpoint(IntakeConstants.WRIST_HANDOFF);
        } else if (intake.getIntakeSlidePos() > IntakeConstants.WAYPOINT_STOW && !controller2.triangle.isPressed()){
            intake.setWristSetpoint(IntakeConstants.WRIST_INTAKE);
        } else if (controller2.triangle.isPressed()) {
            intake.setWristSetpoint(180.0);
        }

        //ARM SETPOINTS
        if (controller2.square.isPressed()) {
            elevator.setArmSetpoint(ElevatorConstants.ARM_BASKET);
        } else if (controller2.circle.isPressed()) {
            elevator.setArmSetpoint(ElevatorConstants.ARM_HANDOFF);
        } else {
            elevator.setArmSetpoint(ElevatorConstants.ARM_STOW);
        }

        //INTAKE WHEEL
        intake.runIntake(controller2.leftTrigger.value() - controller2.rightTrigger.value());
        elevator.runArmGrabber(controller2.rightTrigger.value() - controller2.leftTrigger.value());

        intake.periodic();

        drivetrain.drive(controller1.leftStick.toVector(), controller1.rightStick.toVector().x, false);

        telemetry.addData("Robot Pose", drivetrain.getPose().toString());
        telemetry.addData("Yaw Rate", drivetrain.getYawVelo() );
        drivetrain.sendDashboardPacket(dashboard);
        telemetry.addLine(elevator.getTelemetry());
        telemetry.addLine(intake.getTelemetry());
    }
}
