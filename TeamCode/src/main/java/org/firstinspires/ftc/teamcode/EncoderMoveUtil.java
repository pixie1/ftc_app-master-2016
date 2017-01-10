package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class EncoderMoveUtil {
    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;

     ModernRoboticsI2cGyro sensorGyro;
    public Telemetry telemetry;
    int counter;
    boolean correctPos;

    final int ENCODER_TICKS_NEVEREST= 1120;
    final int ENCODER_TICKS_TETRIX= 1440;
    final double INCH_TO_CM= 2.54;
    final int WHEEL_DIAMETER=4; //in inches

    public EncoderMoveUtil(DcMotor motorFrontRight, DcMotor motorBackLeft, DcMotor motorBackRight,
                           DcMotor motorFrontLeft, Telemetry telemetry, ModernRoboticsI2cGyro sensorMRGyro){
        this.motorBackLeft=motorBackLeft;
        this.motorBackRight=motorBackRight;
        this.motorFrontLeft=motorFrontLeft;
        this.motorFrontRight=motorFrontRight;
        this.motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.telemetry=telemetry;
        this.sensorGyro=sensorMRGyro;
    }
    public int cmToEncoderTicks(double cm) {
        double d = INCH_TO_CM * WHEEL_DIAMETER;
        double rotationConstant = d *Math.PI;
        Double doubleEncoderTicks = (cm * (1 / rotationConstant)) * ENCODER_TICKS_NEVEREST;
        return doubleEncoderTicks.intValue();
    }
    public void stopMotors() {
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }
    public  int angleToEncoderTicks(double turnAmount) {
        double s = ((turnAmount) / 360 * (2 * Math.PI)) * (17.51 / 2);
        double cir = WHEEL_DIAMETER * Math.PI; //4in wheels diameter
        double numOfRotations = s / cir;
        Double encoderTicks = numOfRotations * ENCODER_TICKS_NEVEREST; //1440
        int returnEncoderTicks = (encoderTicks.intValue())*2;
        return returnEncoderTicks;
    }
    public void forward(double disInCm, double speed) {
        int current = motorFrontRight.getCurrentPosition();
        int disInEncoderTicks = cmToEncoderTicks(disInCm);
        telemetry.addData("currentEncoderValue", current);
        telemetry.addData("disInEncoderTicks", disInEncoderTicks);
        telemetry.update();
        motorFrontLeft.setPower(-speed);
        motorFrontRight.setPower(-speed);
        motorBackLeft.setPower(-speed);
        motorBackRight.setPower(-speed);
        while (Math.abs(Math.abs(motorFrontRight.getCurrentPosition())-Math.abs(current)) < Math.abs(disInEncoderTicks)) {
            telemetry.addData("Centimeters:", disInCm);
            telemetry.addData("Encoder Ticks:", disInEncoderTicks);
            telemetry.addData("Left Encoder at:", motorFrontLeft.getCurrentPosition());
            telemetry.addData("Right Encoder at:", motorFrontRight.getCurrentPosition());
            telemetry.addData("Back Left Encoder at:", motorBackLeft.getCurrentPosition());
            telemetry.addData("Back Right Encoder at:", motorBackRight.getCurrentPosition());
            telemetry.update();

        }
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }
    public void backward(double disInCm, double speed) {
        int current = motorFrontRight.getCurrentPosition();
        int disInEncoderTicks = cmToEncoderTicks(disInCm);
        motorFrontLeft.setPower(speed);
        motorFrontRight.setPower(speed);
        motorBackLeft.setPower(speed);
        motorBackRight.setPower(speed);
        while (Math.abs(Math.abs(motorFrontRight.getCurrentPosition())-Math.abs(current)) < Math.abs(disInEncoderTicks)) {
            telemetry.addData("Centimeters:", disInCm);
            telemetry.addData("Encoder Ticks:", disInEncoderTicks);
            telemetry.addData("Front Left Encoder at:", motorFrontLeft.getCurrentPosition());
            telemetry.addData("Right Encoder at:", motorFrontRight.getCurrentPosition());
            telemetry.addData("Back Left Encoder at:", motorBackLeft.getCurrentPosition());
            telemetry.addData("Back Right Encoder at:", motorBackRight.getCurrentPosition());
            telemetry.update();
        }
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }
    public void turnGyroPrecise(int targetRelativeHeading, double speed) {

        int angleCurrent = sensorGyro.getIntegratedZValue();
        int targetHeading = angleCurrent + targetRelativeHeading;
        telemetry.addData("HeadingCurrent", angleCurrent);
        telemetry.addData("Target", targetHeading);
        telemetry.update();
        boolean right = false;
        boolean left = false;
        while (sensorGyro.getIntegratedZValue() > targetHeading || sensorGyro.getIntegratedZValue() < targetHeading) {
            if (sensorGyro.getIntegratedZValue() > targetHeading) {
                if (right) {
                    stopMotors();
                    right = false;
                    left = true;
                }
                motorBackLeft.setPower(speed);
                motorBackRight.setPower(-speed);
                motorFrontLeft.setPower(speed);
                motorFrontRight.setPower(-speed);
                telemetry.addData("HeadingCurrent", sensorGyro.getIntegratedZValue());
                telemetry.addData("Target", targetRelativeHeading);
                telemetry.update();
            } else if (sensorGyro.getIntegratedZValue() < targetHeading) {
                if (left) {
                    stopMotors();
                    left = false;
                    right = true;
                }
                motorBackLeft.setPower(-speed);
                motorBackRight.setPower(speed);
                motorFrontLeft.setPower(-speed);
                motorFrontRight.setPower(speed);
                telemetry.addData("HeadingCurrent", sensorGyro.getIntegratedZValue());
                telemetry.addData("Target", targetRelativeHeading);
                telemetry.update();
            } else break;
        }
    }
    public void turnGyro(int targetRelativeHeading, double speed){

        int angleCurrent = sensorGyro.getIntegratedZValue();
        int targetHeading=angleCurrent+targetRelativeHeading;
        telemetry.addData("HeadingCurrent", angleCurrent);
        telemetry.addData("Target", targetHeading);
        telemetry.update();
        boolean right = false;
        boolean left= false;
        while (sensorGyro.getIntegratedZValue()>targetHeading+1 || sensorGyro.getIntegratedZValue()<targetHeading-1){
            if (sensorGyro.getIntegratedZValue()>targetHeading+1){
                if (right) {
                    stopMotors();
                    right=false;
                    left=true;
                }
               motorBackLeft.setPower(speed);
               motorBackRight.setPower(-speed);
               motorFrontLeft.setPower(speed);
               motorFrontRight.setPower(-speed);
               telemetry.addData("HeadingCurrent", sensorGyro.getIntegratedZValue());
               telemetry.addData("Target",targetRelativeHeading);
               telemetry.update();
           } else if (sensorGyro.getIntegratedZValue()<targetHeading-1){
                if (left) {
                    stopMotors();
                    left=false;
                    right=true;
                }
               motorBackLeft.setPower(-speed);
               motorBackRight.setPower(speed);
               motorFrontLeft.setPower(-speed);
               motorFrontRight.setPower(speed);
               telemetry.addData("HeadingCurrent", sensorGyro.getIntegratedZValue());
               telemetry.addData("Target",targetRelativeHeading);
               telemetry.update();
           } else break;
       }
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);

        telemetry.addData("TurnDone",0);
        telemetry.update();
    }
}
