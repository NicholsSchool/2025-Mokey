package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.ElevatorConstants;
import org.firstinspires.ftc.teamcode.math_utils.Angles;
import org.firstinspires.ftc.teamcode.math_utils.LerpPathPlanning;
import org.firstinspires.ftc.teamcode.math_utils.Point;
import org.firstinspires.ftc.teamcode.math_utils.Vector;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;

import java.util.concurrent.TimeUnit;

/**
 * Testing Auto for Lerp
 */
@Autonomous(name="Lerp Testing", group="Testing")
public class Auto extends LinearOpMode{

    @Override
    public void runOpMode() {
        Drivetrain drivetrain = new Drivetrain(hardwareMap, 0, 0, 0, false);
        drivetrain.setFloat();
        ElapsedTime time = new ElapsedTime();
        Elevator elevator = new Elevator(hardwareMap, drivetrain::getElevatorPosition);

        waitForStart();
//        drivetrain.update();
//        while((Math.abs(24 - drivetrain.getPose(). x) > 2) && (Math.abs(24 - drivetrain.getPose().y) > 2)){
//            drivetrain.update();
//            double mag = Math.hypot(24 - drivetrain.getPose().x, 24 - drivetrain.getPose().y) * 3;
//            drivetrain.drive(new Vector((24 - drivetrain.getPose().x) / mag, (24 - drivetrain.getPose().y) / mag), 0, true);
////            drivetrain.drive(new Vector(0.1,0.1), 0, true);
//            telemetry.addData("x", drivetrain.getPose().x);
//            telemetry.addData("y", drivetrain.getPose().y);
//            telemetry.addData("driveX", (24 - drivetrain.getPose().x) / mag);
//            telemetry.addData("driveY", (24 - drivetrain.getPose().y) / mag);
//            telemetry.update();
//        }
        drivetrain.setTargetHeading(drivetrain.getPose().angle);
        elevator.setSetpoint(ElevatorConstants.WAYPOINT_READY);
        while(time.time(TimeUnit.SECONDS) < 5.0){
            drivetrain.drive(new Vector(0.0,0.7), 0, true);
            elevator.periodic();
            drivetrain.update();
        }
        time.reset();
        elevator.setSetpoint(ElevatorConstants.WAYPOINT_PULL);
        while(time.time(TimeUnit.SECONDS) < 1.5){
            elevator.periodic();
        }
        time.reset();
        elevator.setSetpoint(ElevatorConstants.WAYPOINT_READY);
        while(time.time(TimeUnit.SECONDS) < 1.5){
            elevator.periodic();
        }
        time.reset();
        while(time.time(TimeUnit.SECONDS) < 3.0){
            drivetrain.drive(new Vector(0.8,-0.5), 0, true);
            drivetrain.update();
        }
        time.reset();
        elevator.setSetpoint(ElevatorConstants.WAYPOINT_ZERO);
        while(time.time(TimeUnit.SECONDS) < 2){
            elevator.periodic();
        }
    }
}