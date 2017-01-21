/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;


import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robocol.TelemetryMessage;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 *
 * This is an example LinearOpMode that shows how to use
 * a Modern Robotics Color Sensor.
 *
 * The op mode assumes that the color sensor
 * is configured with a name of "sensor_color".
 *
 * You can use the X button on gamepad1 to toggle the LED on and off.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous
public class MasterAutoBeaconBlue extends LinearOpMode {
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
        final int RBL = 26; // ROBOT LENGTH
        final double MATLIGHT = 0.7; // MAT REFLECTED LIGHT
        final double WHITELIGHT = 1; // LINE REFLECTED LIGHT
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

        //lineSensor = hardwareMap.colorSensor.get("sensor_line");
        //lineSensor.setI2cAddress(new I2cAddr(0x3a));


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
        AutonomousUtil = new AutonomousUtil(catcherL, buttonbashL, motorFrontRight, motorBackLeft, motorBackRight, motorFrontLeft, telemetry, sensorGyro, launchR, launchL, buttonbashR, catcherR);

        colorSensor.enableLed(false);
        lightSensor.enableLed(true);

        waitForStart();

        //encoderMoveUtil.backward(18, 0.3);        //travel to shooting distance
        encoderMoveUtil.backward(80, 0.3);
        //launch particle code
        lineLookTime.reset();
        AutonomousUtil.launchL.setPower(1);
        AutonomousUtil.launchR.setPower(-1);
        while(lineLookTime.seconds()<= .5){}
        catcherL.setPosition(1);
        catcherR.setPosition(0);
        while(lineLookTime.seconds()<= 2){}
        AutonomousUtil.launchL.setPower(0);
        AutonomousUtil.launchR.setPower(0);
        catcherL.setPosition(.5);
        catcherR.setPosition(.5);
       // encoderMoveUtil.backward(30, 0.3);        //travel to middle of (3,2)
        encoderMoveUtil.turnGyro(90, 0.2);
        encoderMoveUtil.backward(60, 0.5);
        int gyroturn2 = 45 - sensorGyro.getIntegratedZValue();
        telemetry.addData("Z VAL", sensorGyro.getIntegratedZValue());
        telemetry.addData("GYROTURN", gyroturn2);
        encoderMoveUtil.turnGyro(gyroturn2,0.2);
        //encoderMoveUtil.turnGyro(60, 0.15);             //sets course to the line
        //encoderMoveUtil.backward(85, 0.3);              //travels to line
        lineLookTime.reset();

        while (lightSensor.getLightDetected() < MATLIGHT && lineLookTime.seconds() < 10) {
            telemetry.addData("LIGHT", lightSensor.getLightDetected());
            telemetry.update();
            motorBackLeft.setPower(0.15);
            motorBackRight.setPower(0.15);
            motorFrontLeft.setPower(0.15);
            motorFrontRight.setPower(0.15);
        }
        lineLookTime.reset();
        encoderMoveUtil.forward(10,0.2);
        /*
        while (lightSensor.getLightDetected() < MATLIGHT+0.2 ) {
            telemetry.addData("LIGHT", lightSensor.getLightDetected());
            telemetry.update();
            motorBackLeft.setPower(-0.1);
            motorBackRight.setPower(-0.1);
            motorFrontLeft.setPower(-0.1);
            motorFrontRight.setPower(-0.1);
        }
        */
        telemetry.addData("LINE FOUND", 0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        buttonbashL.setPosition(1);
        buttonbashR.setPosition(0.3);
        gyroturn2 = 90 - sensorGyro.getIntegratedZValue();
        telemetry.addData("Z VAL", sensorGyro.getIntegratedZValue());
        telemetry.addData("GYROTURN", gyroturn2);
        encoderMoveUtil.turnGyro(gyroturn2,0.2);

        gyroturn2 = 90 - sensorGyro.getIntegratedZValue();
        telemetry.addData("Z VAL", sensorGyro.getIntegratedZValue());
        telemetry.addData("GYROTURN", gyroturn2);
        encoderMoveUtil.turnGyroPrecise(gyroturn2,0.175);
        while (rangeSensor.cmUltrasonic() < 30 && rangeSensor.cmUltrasonic() > 10) {// && rangeSensor.cmUltrasonic() < 100) {
                motorFrontLeft.setPower(0.2);
                motorFrontRight.setPower(0.2);
                motorBackLeft.setPower(0.2);
                motorBackRight.setPower(0.2);
            //if (lightSensor.getLightDetected() > MATLIGHT) {
            //    motorFrontLeft.setPower(0.5);
            //    motorFrontRight.setPower(-0.5);
            //    motorBackLeft.setPower(0.5);
            //    motorBackRight.setPower(-0.5);
            //} else {ewqq11
            //    motorFrontLeft.setPower(-0.5);
            //    motorFrontRight.setPower(0.5);
            //    motorBackLeft.setPower(-0.5);
            //    motorBackRight.setPower(0.5);
            }
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }
        /*
//    encoderMoveUtil.turnGyro(10,0.5);
        while (rangeSensor.cmUltrasonic() > 5) {
            motorFrontLeft.setPower(0.1);
            motorFrontRight.setPower(0.1);
            motorBackLeft.setPower(0.1);
            motorBackRight.setPower(0.1);
        }
        buttonbashR.setPosition(0.3);
        if (colorSensor.red() >= 1) {
            telemetry.addData("RED", "");
            buttonbashR.setPosition(0.7);
            buttonbashL.setPosition(1);
        }
        if (colorSensor.blue() >= 1) {
            telemetry.addData("BLUE", "");
            buttonbashL.setPosition(0);
            buttonbashR.setPosition(0.3);

        }
        motorFrontLeft.setPower(0.1);
        motorFrontRight.setPower(0.1);
        motorBackLeft.setPower(0.1);
        motorBackRight.setPower(0.1);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    */
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
//       motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//      motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//      motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//      motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorBackLeft.setMaxSpeed(2500);
        motorFrontLeft.setMaxSpeed(2500);
        motorBackRight.setMaxSpeed(2500);
        motorFrontRight.setMaxSpeed(2500);
    }
}
