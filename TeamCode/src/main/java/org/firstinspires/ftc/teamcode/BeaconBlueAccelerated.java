package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class BeaconBlueAccelerated extends LinearOpMode {
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
        //AutonomousUtil.beaconLauncher(false);
        //AutonomousUtil.beacon(false);
        //AutonomousUtil.beaconRampEnd(false);
        //STARTING CODE
        encoderMoveUtil.backward(60, SPEED);
        //launch particle code
        lineLookTime.reset();
        AutonomousUtil.launchL.setPower(1);
        AutonomousUtil.launchR.setPower(-1);
        while(lineLookTime.seconds()<= .5){}
        catcherL.setPosition(1);
        catcherR.setPosition(0);
        while(lineLookTime.seconds()<= 2.5){}
        AutonomousUtil.launchL.setPower(0);
        AutonomousUtil.launchR.setPower(0);
        catcherL.setPosition(.5);
        catcherR.setPosition(.5);
        //encoderMoveUtil.backward(20, 0.3);
        // Balls are launched, moving to next step
        encoderMoveUtil.turnGyro(80,TURNSPEED*2);//Turn to white line
        encoderMoveUtil.turnGyroPrecise(10,TURNSPEED);
        encoderMoveUtil.backward(55, SPEED);
        int gyroturn2 = 45 - sensorGyro.getIntegratedZValue();
        telemetry.addData("Z VAL", sensorGyro.getIntegratedZValue());
        telemetry.addData("GYROTURN", gyroturn2);
        encoderMoveUtil.turnGyro(gyroturn2,TURNSPEED); //TURNING TOWARDS LINE
        encoderMoveUtil.backward(30, SPEED);
        lineLookTime.reset();
        //LOOKING FOR LINE
        while (lightSensor.getLightDetected() < MATLIGHT && lineLookTime.seconds() < 5) {
            telemetry.addData("LIGHT", lightSensor.getLightDetected());
            telemetry.update();
            motorBackLeft.setPower(LIGHTSPEED);
            motorBackRight.setPower(LIGHTSPEED);
            motorFrontLeft.setPower(LIGHTSPEED);
            motorFrontRight.setPower(LIGHTSPEED);
        }
        // Line detected, moving backward and turning to beacon
        lineLookTime.reset();
        telemetry.addData("LINE FOUND", 0);
        encoderMoveUtil.forward(10,SPEEDSLOW);
        encoderMoveUtil.stopMotors();
        buttonbashL.setPosition(1);
        buttonbashR.setPosition(0.3);
        gyroturn2 = 90 - sensorGyro.getIntegratedZValue();
        telemetry.addData("Z VAL", sensorGyro.getIntegratedZValue());
        telemetry.addData("GYROTURN", gyroturn2);
        encoderMoveUtil.turnGyro(gyroturn2,TURNSPEED);//TURNING AND FACING BEACON
        gyroturn2 = 90 - sensorGyro.getIntegratedZValue();
        telemetry.addData("Z VAL", sensorGyro.getIntegratedZValue());
        telemetry.addData("GYROTURN", gyroturn2);
        encoderMoveUtil.turnGyroPrecise(gyroturn2,TURNSPEED);
        beaconRunup(false);
        encoderMoveUtil.forward(20,SPEEDSLOW);
        buttonbashL.setPosition(1);
        buttonbashR.setPosition(0.3);
        //STARTING BEACON#2
        encoderMoveUtil.turnGyro(-80,TURNSPEED*2);//Turn to white line
        encoderMoveUtil.turnGyroPrecise(-10,TURNSPEED);
        encoderMoveUtil.backward(80,SPEED);
        lineLookTime.reset();
        while (lightSensor.getLightDetected() < MATLIGHT && lineLookTime.seconds() < 10) {
            telemetry.addData("LIGHT", lightSensor.getLightDetected());
            telemetry.update();
            motorBackLeft.setPower(LIGHTSPEED);
            motorBackRight.setPower(LIGHTSPEED);
            motorFrontLeft.setPower(LIGHTSPEED);
            motorFrontRight.setPower(LIGHTSPEED);
        }
        encoderMoveUtil.forward(10,SPEEDSLOW);
        gyroturn2 = 80 - sensorGyro.getIntegratedZValue();
        encoderMoveUtil.turnGyro(gyroturn2,TURNSPEED*2);
        telemetry.addData("Z VAL", sensorGyro.getIntegratedZValue());
        telemetry.addData("GYROTURN", gyroturn2);
        gyroturn2 = 90 - sensorGyro.getIntegratedZValue();
        encoderMoveUtil.turnGyroPrecise(gyroturn2,TURNSPEED);//turning to face beacon
        beaconRunup(false);
        encoderMoveUtil.forward(20,SPEED);
        encoderMoveUtil.turnGyro(135,0.5);
        encoderMoveUtil.backward( 110,0.8);

    }
    final double SPEED = 0.75; //Faster Speed Constant (Use on long straights)
    final double SPEEDSLOW = 0.3; //Slower Speed Constant (Use on short straights)
    final double TURNSPEED = 0.2; //Turning Speed Constant (Lower for precision, higher for speed)
    final double LIGHTSPEED = 0.2; //Ultrasonic and Lightsensor Speed Constants
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
        if (rangeSensor.cmUltrasonic() < 20){
            encoderMoveUtil.forward(10,SPEEDSLOW);
        }
        while (rangeSensor.cmUltrasonic() < 100 && rangeSensor.cmUltrasonic() > 20) { //Run up to beacon
            motorFrontLeft.setPower(LIGHTSPEED);
            motorFrontRight.setPower(LIGHTSPEED);
            motorBackLeft.setPower(LIGHTSPEED);
            motorBackRight.setPower(LIGHTSPEED);
            telemetry.addData("DIST:", rangeSensor.cmUltrasonic());
            telemetry.update();
        }
        telemetry.addData("BEACON REACHED","");
        buttonbashR.setPosition(0.7);
        encoderMoveUtil.stopMotors();
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        encoderMoveUtil.backward(5,SPEEDSLOW);//Move a bit forward
        if (red==true) { //Based on autonomous, look for the correct colour and hit the right button
            if (colorSensor.red() >= 1) {
                telemetry.addData("RED", "");
                buttonbashR.setPosition(0.9);
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
                buttonbashR.setPosition(0.9);
            }
        }
        encoderMoveUtil.forward(6,SPEEDSLOW);
        encoderMoveUtil.backward(11,SPEEDSLOW); // Beacon has been hit, waiting then stopping
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
