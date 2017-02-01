package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class BeaconBlueCenter extends LinearOpMode {
    EncoderMoveUtil encoderMoveUtil;
    AutonomousUtil AutonomousUtil;

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor launchR;
    DcMotor launchL;
    Servo buttonbashL;
    Servo buttonbashR;
    Servo catcherL;
    Servo catcherR;

    ModernRoboticsI2cGyro sensorGyro;
    ColorSensor colorSensor;
    ModernRoboticsI2cRangeSensor rangeSensor;
    OpticalDistanceSensor lightSensor;

    @Override
    public void runOpMode() {
        final double MATLIGHT = 0.7; // MAT REFLECTED LIGHT
        ElapsedTime lineLookTime = new ElapsedTime();

        initMotors();
        buttonbashL = hardwareMap.servo.get("servo_3");
        buttonbashR = hardwareMap.servo.get("servo_4");
        catcherL = hardwareMap.servo.get("servo_1");
        catcherR = hardwareMap.servo.get("servo_2");

        lightSensor = hardwareMap.opticalDistanceSensor.get("light_sensor");
        sensorGyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
        colorSensor = hardwareMap.colorSensor.get("sensor_color");

        buttonbashL.setPosition(0);
        buttonbashR.setPosition(0.7);
        catcherL.setPosition(0.5);
        catcherR.setPosition(0.5);

        sensorGyro.calibrate();
        while (sensorGyro.isCalibrating()) {
            telemetry.addData("gyro sensor is calibrating", "0");
            telemetry.update();
        }
        telemetry.addData("Initialization done", "0");
        telemetry.update();

        encoderMoveUtil = new EncoderMoveUtil(motorFrontRight, motorBackLeft, motorBackRight, motorFrontLeft,
                telemetry, sensorGyro);
        AutonomousUtil = new AutonomousUtil(catcherL, buttonbashL, motorFrontRight, motorBackLeft, motorBackRight, motorFrontLeft, telemetry, sensorGyro, launchR, launchL, buttonbashR, catcherR, rangeSensor, colorSensor, lightSensor);

        colorSensor.enableLed(false);
        lightSensor.enableLed(true);

        waitForStart();
        AutonomousUtil.beaconLauncher(false);
        AutonomousUtil.beacon(false);
        AutonomousUtil.beaconBallEnd(false);
    }
    private void initMotors() {
        motorFrontRight = hardwareMap.dcMotor.get("motor_1");
        motorBackRight = hardwareMap.dcMotor.get("motor_2");
        motorFrontLeft = hardwareMap.dcMotor.get("motor_3");
        motorBackLeft = hardwareMap.dcMotor.get("motor_4");
        launchR = hardwareMap.dcMotor.get("motor_5");
        launchL = hardwareMap.dcMotor.get("motor_6");

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorBackLeft.setMaxSpeed(2500);
        motorFrontLeft.setMaxSpeed(2500);
        motorBackRight.setMaxSpeed(2500);
        motorFrontRight.setMaxSpeed(2500);
    }
    public void beaconRunup(boolean red) {
        while (rangeSensor.cmUltrasonic() < 100 && rangeSensor.cmUltrasonic() > 20) { //Run up to beacon
            motorFrontLeft.setPower(0.15);
            motorFrontRight.setPower(0.15);
            motorBackLeft.setPower(0.15);
            motorBackRight.setPower(0.15);
            telemetry.addData("DIST:", rangeSensor.cmUltrasonic());
            telemetry.update();
        }
        telemetry.addData("BEACON REACHED","");
        buttonbashR.setPosition(0.7);
        encoderMoveUtil.stopMotors();
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        encoderMoveUtil.backward(5,0.15);//Move a bit forward
        if (red==true) { //Based on autonomous, look for the correct colour and hit the right button
            if (colorSensor.red() >= 1) {
                telemetry.addData("RED", "");
                buttonbashR.setPosition(0.8);
                buttonbashL.setPosition(1);
            }
            if (colorSensor.blue() >= 1) {
                telemetry.addData("BLUE", "");
                buttonbashL.setPosition(0);
                buttonbashR.setPosition(0.2);
            }
        } else {
            if (colorSensor.red() >= 1) {
                telemetry.addData("RED", "");
                buttonbashR.setPosition(0.2);
                buttonbashL.setPosition(0);
            }
            if (colorSensor.blue() >= 1) {
                telemetry.addData("BLUE", "");
                buttonbashL.setPosition(1);
                buttonbashR.setPosition(0.8);
            }
        }
        encoderMoveUtil.forward(5,0.15);
        encoderMoveUtil.backward(11,0.15); // Beacon has been hit, waiting then stopping
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
