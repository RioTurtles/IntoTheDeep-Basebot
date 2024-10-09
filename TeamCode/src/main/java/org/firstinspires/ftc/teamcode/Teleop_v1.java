package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class Teleop_v1 extends LinearOpMode {
    Project1Hardware robot;
    States state;
    Gamepad gamepad;
    Gamepad lastGamepad;
    double direction_x, direction_y, pivot, heading;

    ElapsedTime timer1;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Project1Hardware(hardwareMap);
        state = States.INTAKE_READY;
        gamepad = new Gamepad();
        lastGamepad = new Gamepad();
        timer1 = new ElapsedTime();

        waitForStart();
        robot.imu.resetYaw();
        timer1.reset();
        while (opModeIsActive()) {
            lastGamepad.copy(gamepad);
            gamepad.copy(gamepad1);
            direction_x = -gamepad1.left_stick_x;
            direction_y = gamepad1.left_stick_y;
            pivot = gamepad1.right_stick_x * 0.8;
            heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            if (gamepad.touchpad) robot.imu.resetYaw();

            if (state == States.INTAKE_READY) {
                if (timer1.milliseconds() > 300) robot.clawOpen();

                if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                    if (robot.clawOpen) robot.clawClose(); else robot.clawOpen();
                }

                if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                    timer1.reset();
                    state = States.TRANSFER_ONE;
                }
            }

            if (state == States.TRANSFER_ONE) {
                robot.clawClose();
                if (timer1.milliseconds() > 300) robot.armUp();

                if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                    robot.armDown();
                    state = States.INTAKE_READY;
                    timer1.reset();
                }

                if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                    state = States.SCORING_READY;
                }
            }

            if (state == States.SCORING_READY) {
                robot.setSliderPosition();
                if (robot.isSliderInPosition()) {
                    if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                        if (robot.clawOpen) robot.clawClose(); else robot.clawOpen();
                    }
                }

                if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                    robot.setSliderPosition(1, 0);
                    state = States.TRANSFER_TWO;
                    timer1.reset();
                }
            }

            if (state == States.TRANSFER_TWO) {
                robot.setSliderPosition(1, 0);
                robot.clawOpen();
                if (robot.isSliderInPosition()) state = States.INTAKE_READY;

                if (gamepad.left_bumper && !gamepad.left_bumper) {
                    state = States.SCORING_READY;
                    timer1.reset();
                }
            }

            if (gamepad.triangle) robot.mode = 1;
            if (gamepad.circle) robot.mode = 2;
            if (gamepad.cross) robot.height = 1;
            if (gamepad.square) robot.height = 2;

            telemetry.addData("State", state);
            telemetry.addData("Mode", robot.mode);
            telemetry.addData("Height", robot.height);
            telemetry.update();
            robot.drivetrain.remote(direction_y, -pivot, direction_x, heading);
        }
    }

    enum States {
        INTAKE_READY,
        TRANSFER_ONE,
        SCORING_READY,
        TRANSFER_TWO
    }
}
