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

public class Project1Hardware {
    DcMotor motorFL, motorFR;
    DcMotor motorBL, motorBR;
    DcMotorEx sliderL, sliderR;
    ServoImplEx claw;
    ServoImplEx armL, armR;
    MecanumDrive drivetrain;
    IMU imu;

    int mode, height;
    boolean clawOpen, armUp, middlePosition;

    public Project1Hardware(HardwareMap hardwareMap) {
        init(hardwareMap);
        reset();
        mode = 1;
        height = 2;
        clawOpen = false;
        armUp = false;
        middlePosition = false;
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
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);

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
            BLPower = power * (cos/max) - pivot;
            BRPower = power * (sin/max) + pivot;

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
    public void clawOpen() {claw.setPosition(0.45); clawOpen = true;}
    public void clawClose() {claw.setPosition(0.16); clawOpen = false;}

    public void armUp() {
        if (middlePosition) {
            armL.setPosition(0.25);
            armR.setPosition(0.25);
        } else {
            armL.setPosition(0.44);
            armR.setPosition(0.44);
        }
        armUp = true;
    }

    public void armDown() {
        if (middlePosition) {
            armL.setPosition(0.14);
            armR.setPosition(0.14);
        } else {
            armL.setPosition(0.16);
            armR.setPosition(0.16);
        }
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
            case 1:
                switch (mode) {
                    case 1: setSlider(1800); break;  // Low Basket
                    case 2: setSlider(0); break;  // Low Specimen
                }
                break;
            case 2:
                switch (mode) {
                    case 1: setSlider(3360); break;  // High Basket
                    case 2: setSlider(1400); break;  // High Specimen
                }
                break;
            case 3:
                assert mode == 2; setSlider(670); break;  // Confirm specimen
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

    public String mode() {
        switch (mode) {
            case 1: return "Sample";
            case 2: return "Specimen";
        }
        return "ERROR";
    }

    public String height() {
        switch (height) {
            case 1: return "Low";
            case 2: return "High";
        }
        return "ERROR";
    }

    public String sliderLeftInfo() {
        return sliderL.getPower() + " | "
                + sliderL.getCurrentPosition() + " (T: " + sliderL.getTargetPosition() + " | R: "
                + Math.abs(sliderL.getCurrentPosition() - sliderL.getTargetPosition()) + ")";
    }

    public String sliderRightInfo() {
        return sliderR.getPower() + " | "
                + sliderR.getCurrentPosition() + " (T: " + sliderR.getTargetPosition() + " | R: "
                + Math.abs(sliderR.getCurrentPosition() - sliderR.getTargetPosition()) + ")";
    }
}
