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

            // Start of robot, nothing possessed
            if (state == States.INIT) {
                robot.armUp();
                robot.clawClose();
                robot.setSliderPosition(1, 0);

                if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                    timer1.reset();
                    state = States.INTAKE_READY;
                }
            }

            // Ready to grab samples or specimens
            if (state == States.INTAKE_READY) {
                if (timer1.milliseconds() > 300) robot.clawOpen();
                robot.armDown();

                if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                    robot.clawOpen();
                } else robot.clawClose();

                if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                    timer1.reset();
                    state = States.INIT;
                }

                if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                    timer1.reset();
                    state = States.TRANSFER;
                }
            }

            // Samples possessed, arm has gone up, going to scoring position
            if (state == States.TRANSFER) {
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

            // Slider is raised, ready for scoring
            if (state == States.SCORING_READY) {
                robot.setSliderPosition();

                if (robot.isSliderInPosition()) {
                    if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                        //if (robot.clawOpen) robot.clawClose(); else robot.clawOpen();
                        robot.clawOpen();
                    } else robot.clawClose();
                }

                if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                    robot.setSliderPosition(1, 0);
                    state = States.RETURN;
                    timer1.reset();
                }
            }

            // Finished scoring, return to INIT
            if (state == States.RETURN) {
                robot.setSliderPosition(1, 0);
                robot.clawClose();

                if (robot.isSliderInPosition() && gamepad.right_bumper && !lastGamepad.right_bumper) {
                    state = States.INTAKE_READY;
                }

                if (gamepad.left_bumper && !gamepad.left_bumper) {
                    state = States.SCORING_READY;
                    timer1.reset();
                }
            }

            if (gamepad.square) robot.mode = 1;
            if (gamepad.circle) robot.mode = 2;
            if (gamepad.cross) robot.height = 1;
            if (gamepad.triangle) robot.height = 2;

            if (gamepad.dpad_left && !lastGamepad.dpad_left) {
                timer1.reset();
                state = States.RIGGING;
            }

            if (state == States.RIGGING) {
                robot.setSlider(1000);

                if (gamepad.dpad_down) {
                    robot.sliderL.setPower(-1);
                    robot.sliderR.setPower(-1);
                } else {
                    robot.sliderL.setPower(0);
                    robot.sliderR.setPower(0);
                }

                if (gamepad.dpad_up) {
                    robot.sliderL.setPower(1);
                    robot.sliderR.setPower(1);
                } else {
                    robot.sliderL.setPower(0);
                    robot.sliderR.setPower(0);
                }
            }

            telemetry.addData("State", state);
            telemetry.addData("Mode", robot.mode);
            telemetry.addData("Height", robot.height);

            telemetry.update();
            robot.drivetrain.remote(direction_y, -pivot, direction_x, heading);
        }
    }

    enum States {
        INIT,
        INTAKE_READY,
        TRANSFER,
        SCORING_READY,
        RETURN,
        RIGGING
    }
}