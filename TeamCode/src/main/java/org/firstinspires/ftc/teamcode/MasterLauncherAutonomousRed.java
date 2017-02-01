package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class MasterLauncherAutonomousRed extends LinearOpMode {
    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor launchR;
    DcMotor launchL;
    ModernRoboticsI2cGyro sensorGyro;
    EncoderMoveUtil encoderMoveUtil;
    AutonomousUtil AutonomousUtil;
    Servo buttonbashL;
    Servo buttonbashR;
    Servo catcherL;
    Servo catcherR;

    @Override
    public void runOpMode() {
        final int LoopATime = 1000;
        int LoopARep = 0;
        final int RBL = 26;
        ElapsedTime lineLookTime = new ElapsedTime();

        initMotors();

        buttonbashL = hardwareMap.servo.get("servo_3");
        buttonbashR = hardwareMap.servo.get("servo_4");
        catcherL = hardwareMap.servo.get("servo_1");
        catcherR = hardwareMap.servo.get("servo_2");
        sensorGyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        buttonbashL.setPosition(0.1);
        buttonbashR.setPosition(0.8);
        catcherL.setPosition(0.5);

        sensorGyro.calibrate();
        while (sensorGyro.isCalibrating()) {
            telemetry.addData("gyro sensor is calibrating", "0");
            telemetry.update();
        }
        telemetry.addData("Initialization done", "0");
        telemetry.update();

        encoderMoveUtil = new EncoderMoveUtil(motorFrontRight, motorBackLeft, motorBackRight, motorFrontLeft, telemetry, sensorGyro);
        AutonomousUtil = new AutonomousUtil(catcherL, buttonbashL, motorFrontRight, motorBackLeft, motorBackRight, motorFrontLeft, telemetry, sensorGyro, launchR, launchL, buttonbashR, catcherR, null, null, null);

        waitForStart();

        AutonomousUtil.launch(true); //true = red  false = blue
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

        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorBackLeft.setMaxSpeed(2500);
        motorFrontLeft.setMaxSpeed(2500);
        motorBackRight.setMaxSpeed(2500);
        motorFrontRight.setMaxSpeed(2500);
    }
}
