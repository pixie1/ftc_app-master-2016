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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

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
public class MasterAutoBeacon extends LinearOpMode {
  DcMotor motorFrontRight;
  DcMotor motorFrontLeft;
  DcMotor motorBackRight;
  DcMotor motorBackLeft;
  ModernRoboticsI2cGyro sensorGyro;
  EncoderMoveUtil encoderMoveUtil;
  Servo buttonbashL;
  Servo buttonbashR;
  Servo catcher;
  ColorSensor colorSensor;    // Hardware Device Object
  ColorSensor lineSensor;
  ModernRoboticsI2cRangeSensor rangeSensor;

  @Override
  public void runOpMode() {
    final int LoopATime = 1000;
    int LoopARep = 0;
    final int RBL = 26;

    motorFrontRight = hardwareMap.dcMotor.get("motor_1");
    motorBackRight = hardwareMap.dcMotor.get("motor_2");
    motorFrontLeft = hardwareMap.dcMotor.get("motor_3");
    motorBackLeft = hardwareMap.dcMotor.get("motor_4");
    buttonbashL = hardwareMap.servo.get("servo_3");
    buttonbashR = hardwareMap.servo.get("servo_4");
    catcher = hardwareMap.servo.get("servo_2");
    sensorGyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
    rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
    colorSensor = hardwareMap.colorSensor.get("sensor_color");
    lineSensor = hardwareMap.colorSensor.get("sensor_line");

    lineSensor.setI2cAddress(new I2cAddr(0x3a));

    motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    buttonbashL.setPosition(1);
    buttonbashR.setPosition(0.3);
    catcher.setPosition(0.5);
    sensorGyro.calibrate();
    while (sensorGyro.isCalibrating()) {
      telemetry.addData("gyro sensor is calibrating", "0");
      telemetry.update();
    }
    telemetry.addData("Initialization done", "0");
    telemetry.update();

    encoderMoveUtil = new EncoderMoveUtil(motorFrontRight, motorBackLeft, motorBackRight, motorFrontLeft,
            telemetry, sensorGyro);
    buttonbashL.setPosition(1);
    buttonbashR.setPosition(1);

    colorSensor.enableLed(false);
    lineSensor.enableLed(true);
    waitForStart();
    while (opModeIsActive()) {
      encoderMoveUtil.forward(-90,0.5);
      encoderMoveUtil.turnGyro(60,0.5);
      while (lineSensor.red()<100 && lineSensor.blue()<100 && lineSensor.green()<100 && LoopARep != LoopATime) {
        motorBackLeft.setPower(-0.5);
        motorBackRight.setPower(-0.5);
        motorFrontLeft.setPower(-0.5);
        motorFrontRight.setPower(-0.5);
        LoopARep++;
      }
      while(rangeSensor.cmUltrasonic()>30 && rangeSensor.cmUltrasonic()<100){
        if(lineSensor.red()>50 && lineSensor.blue()>50 && lineSensor.green()>50){
          motorFrontLeft.setPower(0.5);
          motorFrontRight.setPower(-0.5);
          motorBackLeft.setPower(0.5);
          motorBackRight.setPower(-0.5);
        }
        else {
          motorFrontLeft.setPower(-0.5);
          motorFrontRight.setPower(0.5);
          motorBackLeft.setPower(-0.5);
          motorBackRight.setPower(0.5);
        }
      }
      motorFrontLeft.setPower(0);
      motorFrontRight.setPower(0);
      motorBackLeft.setPower(0);
      motorBackRight.setPower(0);
      encoderMoveUtil.turnGyro(10,0.5);
      while(rangeSensor.cmUltrasonic()>5) {
        motorFrontLeft.setPower(-0.5);
        motorFrontRight.setPower(-0.5);
        motorBackLeft.setPower(-0.5);
        motorBackRight.setPower(-0.5);
      }
      if (colorSensor.red() == 1) {
        telemetry.addData("RED","");
        buttonbashR.setPosition(0.3);
      }
      if (colorSensor.blue() == 1) {
        telemetry.addData("BLUE","");
        buttonbashL.setPosition(0);
      }
      /*
      telemetry.addData("Clear", colorSensor.alpha());
      telemetry.addData("Red  ", colorSensor.red());
      telemetry.addData("Green", colorSensor.green());
      telemetry.addData("Blue ", colorSensor.blue());
      telemetry.addData("RGB", colorSensor.argb());
      telemetry.addData("Alpha", colorSensor.alpha());
      telemetry.update();
      */
    }
  }
}
