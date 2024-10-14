package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    boolean intakeInitialActionsFinished, confirmSpecimen, intakeSubmersible;

    ElapsedTime timer1;
    ElapsedTime timer2;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Project1Hardware(hardwareMap);
        state = States.INIT;
        gamepad = new Gamepad();
        lastGamepad = new Gamepad();
        timer1 = new ElapsedTime();
        timer2 = new ElapsedTime();

        waitForStart();
        confirmSpecimen = false;
        intakeSubmersible = true;
        robot.imu.resetYaw();
        timer1.reset();

        while (opModeIsActive()) {
            lastGamepad.copy(gamepad);
            gamepad.copy(gamepad1);

            direction_x = gamepad1.left_stick_x;
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
                    continue;
                }
            }

            // Ready to grab samples or specimens
            if (state == States.INTAKE_READY) {
                robot.middlePosition = intakeSubmersible;
                if (!intakeInitialActionsFinished) {
                    robot.armDown();
                    if (timer1.milliseconds() > 400) {
                        robot.clawOpen();
                        intakeInitialActionsFinished = true;
                    }
                }

                if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                    if (robot.clawOpen) robot.clawClose(); else robot.clawOpen();
                }

                if (gamepad.left_trigger > 0 && !(lastGamepad.left_trigger > 0)) {
                    intakeSubmersible = !intakeSubmersible;
                    robot.middlePosition = intakeSubmersible;
                    if (robot.armUp) robot.armUp();
                }

                if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                    if (robot.armUp) robot.armDown(); else robot.armUp();
                }

                if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                    timer1.reset();
                    intakeInitialActionsFinished = false;
                    state = States.TRANSFER;
                    continue;
                }
            }

            // Samples possessed, arm has gone up, going to scoring position
            if (state == States.TRANSFER) {
                robot.middlePosition = false;
                robot.clawClose();
                if (timer1.milliseconds() > 300) robot.armUp();

                if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                    robot.armDown();
                    state = States.INTAKE_READY;
                    timer1.reset();
                }

                if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                    state = States.SCORING_READY;
                    timer1.reset();
                    continue;
                }

                if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                    if (robot.clawOpen) robot.clawClose(); else robot.clawOpen();
                }
            }

            // Slider is raised, ready for scoring
            if (state == States.SCORING_READY) {
                if (!confirmSpecimen) robot.setSliderPosition();
                else {
                    robot.setSliderPosition(2, 3);
                    if (timer2.milliseconds() > 500) robot.clawOpen();
                }

                if (robot.isSliderInPosition()) {
                    if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                        if (robot.mode == 1) {
                            if (robot.clawOpen) robot.clawClose(); else robot.clawOpen();
                        } else {
                            confirmSpecimen = !confirmSpecimen;
                            if (confirmSpecimen) timer2.reset();
                        }
                    }
                }

                if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                    robot.setSliderPosition(1, 0);
                    confirmSpecimen = false;
                    state = States.TRANSFER;
                    timer1.reset();
                    continue;
                }

                if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                    robot.clawOpen();
                    confirmSpecimen = false;
                    state = States.RETURN;
                    timer1.reset();
                    continue;
                }
            }

            // Finished scoring, return to INIT
            if (state == States.RETURN) {
                robot.setSliderPosition(1, 0);
                robot.clawClose();

                if (robot.isSliderInPosition() && gamepad.right_bumper && !lastGamepad.right_bumper) {
                    robot.middlePosition = intakeSubmersible;
                    state = States.INTAKE_READY;
                }

                if (gamepad.left_bumper && !gamepad.left_bumper) {
                    state = States.SCORING_READY;
                    timer1.reset();
                    continue;
                }
            }

            if (gamepad.square) robot.mode = 1;
            if (gamepad.circle) robot.mode = 2;
            if (gamepad.cross) robot.height = 1;
            if (gamepad.triangle) robot.height = 2;

            if (gamepad.dpad_left && !lastGamepad.dpad_left) {
                robot.setSlider(1750);
                state = States.RIGGING;
                timer1.reset();
                continue;
            }

            if (state == States.RIGGING) {
                robot.middlePosition = false;
                robot.clawClose();
                robot.armUp();
                if (gamepad.dpad_up && !lastGamepad.dpad_up) {
                    robot.setSlider(robot.sliderL.getTargetPosition() - 100);
                }
                if (gamepad.dpad_down) {
                    robot.setSlider(robot.sliderL.getTargetPosition() + 100);
                }
                if (gamepad.dpad_right) robot.setSlider(1750);

                if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                    robot.setSliderPosition(1, 0);
                    state = States.INTAKE_READY;
                    continue;
                }
            }

            // Slider emergency resets
            if (gamepad.options) {
                robot.ascendDownwards(1);  // Ascending downwards raises the sliders
                robot.sliderL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.sliderR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            } else if (gamepad.share) {
                robot.ascendUpwards(1);  // Ascending upwards retracts the sliders
                robot.sliderL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.sliderR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            telemetry.addData("State", state);
            telemetry.addData("Mode", robot.mode());
            telemetry.addData("Height", robot.height());
            telemetry.addLine();
            telemetry.addData("Slider-L", robot.sliderLeftInfo());
            telemetry.addData("Slider-R", robot.sliderRightInfo());

            telemetry.update();
            robot.drivetrain.remote(direction_y, -direction_x, -pivot, heading);
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