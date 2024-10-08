package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Project1Hardware {
    DcMotor motorFL, motorFR;
    DcMotor motorBL, motorBR;
    DcMotorEx sliderL, sliderR;
    ServoImplEx claw;
    ServoImplEx armL, armR;
    MecanumDrive drivetrain;
    IMU imu;

    int mode, height;
    boolean clawOpen, armUp;

    public Project1Hardware(HardwareMap hardwareMap) {
        init(hardwareMap);
        reset();
        mode = 1;
        height = 2;
        clawOpen = false;
        armUp = false;
    }

    private void init(@NonNull HardwareMap hardwareMap) {
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        sliderL = hardwareMap.get(DcMotorEx.class, "sliderL");
        sliderR = hardwareMap.get(DcMotorEx.class, "sliderR");
        claw = hardwareMap.get(ServoImplEx.class, "claw");
        armL = hardwareMap.get(ServoImplEx.class, "armL");
        armR = hardwareMap.get(ServoImplEx.class, "armR");
        imu = hardwareMap.get(IMU.class, "imu");

        //TODO: Change directions
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        )));

        drivetrain = new MecanumDrive();
    }

    private void reset() {
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

        sliderL.setDirection(DcMotorSimple.Direction.FORWARD);
        sliderR.setDirection(DcMotorSimple.Direction.REVERSE);
        sliderL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armL.setDirection(Servo.Direction.REVERSE);
        armR.setDirection(Servo.Direction.FORWARD);
        claw.setDirection(Servo.Direction.FORWARD);

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        )));
    }

    public class MecanumDrive {
        Project1Hardware robot;
        double max, sin, cos, theta, power, vertical, horizontal, pivot, heading;
        double FLPower, FRPower, BLPower, BRPower;

        public MecanumDrive() {this.robot = Project1Hardware.this;}

        /**
         * Classic drivetrain movement method - self explanatory.
         * @param vertical Gamepad's vertical axis (y).
         * @param horizontal Gamepad's horizontal axis (x).
         * @param pivot Gamepad's rotational axis (<code>right_stick_x</code>).
         * @param heading Robot's heading.
         */
        public void remote(double vertical, double horizontal, double pivot, double heading) {
            this.vertical = vertical;
            this.horizontal = horizontal;
            this.pivot = pivot;
            this.heading = heading ;

            theta = 2 * Math.PI + Math.atan2(vertical,horizontal) - heading;
            power = Math.hypot(horizontal, vertical);

            sin = Math.sin(theta - Math.PI/4);
            cos = Math.cos(theta - Math.PI/4);
            max = Math.max(Math.abs(sin), Math.abs(cos));

            FLPower = power * (cos/max) + pivot;
            FRPower = power * sin/max - pivot;
            BLPower = power * -(sin/max) - pivot;
            BRPower = power * -(cos/max) + pivot;

            robot.motorFL.setPower(-FLPower);
            robot.motorFR.setPower(-FRPower);
            robot.motorBL.setPower(BLPower);
            robot.motorBR.setPower(BRPower);

            robot.motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        /** Also mecanum drive ({@link #remote(double, double, double, double) remote()}) but more
         * organised.
         * @param vertical Gamepad's vertical axis (y).
         * @param horizontal Gamepad's horizontal axis (x).
         * @param pivot Gamepad's rotational axis (<code>right_stick_x</code>).
         * @param heading Robot's heading.
         */
        public void remote2(double vertical, double horizontal, double pivot, double heading) {
            robot.motorFL.setDirection(DcMotor.Direction.FORWARD);
            robot.motorFR.setDirection(DcMotor.Direction.REVERSE);
            robot.motorBL.setDirection(DcMotor.Direction.FORWARD);
            robot.motorBR.setDirection(DcMotor.Direction.REVERSE);

            this.vertical = vertical;
            this.horizontal = horizontal;
            this.pivot = pivot;
            this.heading = heading + (Math.PI/2);

            theta = 2 * Math.PI + Math.atan2(vertical, horizontal) - heading;
            power = Math.hypot(horizontal, vertical);

            sin = Math.sin(theta - Math.PI/4);
            cos = Math.cos(theta - Math.PI/4);
            max = Math.max(Math.abs(sin), Math.abs(cos));

            /*
                FLPower = power * (cos/max) + pivot;
                FRPower = power * (sin/max) - pivot;
                BLPower = power * (sin/max) + pivot;
                BRPower = power * (cos/max) - pivot;
            */

            FLPower = power * (cos/max) - pivot;
            FRPower = power * (sin/max) + pivot;
            BLPower = power * (sin/max) - pivot;
            BRPower = power * (cos/max) + pivot;

            robot.motorFL.setPower(FLPower);
            robot.motorFR.setPower(FRPower);
            robot.motorBL.setPower(BLPower);
            robot.motorBR.setPower(BRPower);

            robot.motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        /** Undocumented - copied from MecanumDrive.java */
        public void part1(double theta, double pivot, double power) {
            theta = 2 * Math.PI + (theta / 360 * 2 * Math.PI) - Math.PI / 2;

            sin = Math.sin(theta - Math.PI/4);
            cos = Math.cos(theta - Math.PI/4);
            max = Math.max(Math.abs(sin), Math.abs(cos));

            FLPower = power * (cos/max) + pivot;
            FRPower = power * sin/max - pivot;
            BLPower = power * -(sin/max) - pivot;
            BRPower = power * -(cos/max) + pivot;

            robot.motorFL.setPower(-FLPower);
            robot.motorFR.setPower(-FRPower);
            robot.motorBL.setPower(BLPower);
            robot.motorBR.setPower(BRPower);

            robot.motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        /** Undocumented - copied from MecanumDrive.java */
        public void drive(double target, double power, double pivot, double distance) {

            this.theta = Math.PI + (target * Math.PI/180);
            sin = Math.sin(theta - Math.PI/4);
            cos = Math.cos(theta - Math.PI/4);
            max = Math.max(Math.abs(sin), Math.abs(cos));

            int FL = robot.motorFL.getCurrentPosition();
            int FR = robot.motorFR.getCurrentPosition();
            int BL = robot.motorBL.getCurrentPosition();
            int BR = robot.motorBR.getCurrentPosition();

            double orig = FL;
            double cur = orig;

            while (Math.abs(cur - orig) <= distance) {
                FL = robot.motorFL.getCurrentPosition();
                FR = robot.motorFR.getCurrentPosition();
                BL = robot.motorBL.getCurrentPosition();
                BR = robot.motorBR.getCurrentPosition();

                cur = FL;

                FLPower = power * -(cos/max) + pivot;
                FRPower = power * sin/max + pivot;
                BLPower = power * -(sin/max) + pivot;
                BRPower = power * cos/max + pivot;

                robot.motorFL.setPower(-FLPower);
                robot.motorFR.setPower(-FRPower);
                robot.motorBL.setPower(BLPower);
                robot.motorBR.setPower(BRPower);

                robot.motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
    }

    // Claw tuned
    public void clawOpen() {claw.setPosition(0.2); clawOpen = true;}
    public void clawClose() {claw.setPosition(0); clawOpen = false;}

    public void armUp() {
        armL.setPosition(0.51);
        armR.setPosition(0.51);
        armUp = false;
    }

    public void armDown() {
        // Old value: 0.23
        armL.setPosition(0.28);
        armR.setPosition(0.28);
        armUp = false;
    }

    public void setSlider(int value) {
        sliderL.setTargetPosition(value);
        sliderR.setTargetPosition(value);

        sliderL.setPower(1);
        sliderR.setPower(1);
        sliderL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Sets slider preset positions.
     * @param mode 1 - Basket | 2 - Specimen
     * @param height 0 - Reset | 1 - Low | 2 - High
     */
    public void setSliderPosition(int mode, int height) {
        switch (height) {
            // Reset
            case 0: setSlider(0); break;
            // Low
            case 1:
                switch (mode) {
                    case 1: setSlider(0); break;  // Basket
                    case 2: setSlider(1675); break;  // Specimen
                }
                break;
            // High
            case 2:
                switch (mode) {
                    case 1: setSlider(2100); break;  // Basket
                    case 2: setSlider(3000); break;  // Specimen
                }
                break;
        }
    }

    public void setSliderPosition() {setSliderPosition(this.mode, this.height);}

    public void ascendUpwards(double power) {
        sliderL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sliderR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sliderL.setPower(-Math.abs(power));
        sliderR.setPower(-Math.abs(power));
    }

    public void ascendDownwards(double power) {
        sliderL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sliderR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sliderL.setPower(Math.abs(power));
        sliderR.setPower(Math.abs(power));
    }

    public void ascendStop() {ascendUpwards(0);}

    public boolean isSliderInPosition() {
        return Math.abs(sliderL.getCurrentPosition() - sliderL.getTargetPosition()) <= 5;
    }
}
