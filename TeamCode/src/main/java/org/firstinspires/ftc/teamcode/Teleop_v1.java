package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class Teleop_v1 extends LinearOpMode {
    double direction_y, direction_x, pivot, heading;

    @Override
    public void runOpMode() throws InterruptedException {
        Project1Hardware robot = new Project1Hardware();
        MecanumDrive drivetrain = new MecanumDrive(robot);
        robot.init(hardwareMap, telemetry);
        robot.reset();

        waitForStart();
        robot.imu.resetYaw();
        drivetrain.remote(0,0,0,0);
        while (opModeIsActive()) {
            direction_y = gamepad1.left_stick_y;
            direction_x = -gamepad1.left_stick_x;
            pivot = gamepad1.right_stick_x * 0.8;
            heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            if (gamepad1.touchpad) robot.imu.resetYaw();

            telemetry.update();
            drivetrain.remote(direction_y, -pivot, direction_x, heading);
        }
    }
}
