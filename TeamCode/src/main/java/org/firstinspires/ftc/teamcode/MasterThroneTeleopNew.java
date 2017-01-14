 package org.firstinspires.ftc.teamcode;


 import com.qualcomm.robotcore.eventloop.opmode.OpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.Servo;
 import com.qualcomm.robotcore.util.ElapsedTime;


 /**
 * Created by Karine on 10/27/2015.
 */
@TeleOp
public class MasterThroneTeleopNew extends OpMode {

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor launchR;
    DcMotor launchL;

    Servo catcher;
    Servo buttonBasherL;
     Servo buttonBasherR;

    /**
     * Constructor
     */
    public MasterThroneTeleopNew() {
    }

    @Override
    public void init() {
        //this is where we define all of the motors on the robot.
        motorFrontRight = hardwareMap.dcMotor.get("motor_1");
        motorBackRight = hardwareMap.dcMotor.get("motor_2");
        motorFrontLeft = hardwareMap.dcMotor.get("motor_3");
        motorBackLeft = hardwareMap.dcMotor.get("motor_4");
        launchR = hardwareMap.dcMotor.get("motor_5");
        launchL = hardwareMap.dcMotor.get("motor_6");
        catcher = hardwareMap.servo.get("servo_1");
        buttonBasherL = hardwareMap.servo.get("servo_3");
        buttonBasherR = hardwareMap.servo.get("servo_4");
        // pushl = hardwareMap.servo.get("servo_3");
        //pushr = hardwareMap.servo.get("servo_4");

        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launchL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launchR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        catcher.setPosition(.5);
        buttonBasherR.setPosition(0.8);
        buttonBasherL.setPosition(1);
    }
     //int ButtonState = 0;
     //boolean toggleA = false;
     //boolean toggleDpadL = false;
     //boolean toggleDpadR = false;
     //boolean aPrevStat = false;
     @Override
    public void loop() {
        //Control method #2 Joysticks
         //boolean aCurr = gamepad2.a;


        double n = ((gamepad1.left_stick_x + gamepad1.left_stick_y))/.8;
        double m = (-(gamepad1.left_stick_y - gamepad1.left_stick_x))/.8;
        motorFrontRight.setPower(n);
        motorBackRight.setPower(n);
        motorFrontLeft.setPower(m);
        motorBackLeft.setPower(m);
        if (gamepad2.a) {
            launchL.setPower(1);
            launchR.setPower(-1);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            catcher.setPosition(1);

        }
         if (gamepad2.y) {
             launchL.setPower(0);
             launchR.setPower(0);
             catcher.setPosition(.3);
             //ButtonState = 1;
         }
         if (gamepad2.x) {
             launchL.setPower(0);
             launchR.setPower(0);
             catcher.setPosition(1);
         }
         if(gamepad2.b) {
             catcher.setPosition(.5);
             launchL.setPower(0);
             launchR.setPower(0);
         }
         if(gamepad2.dpad_up) {
             buttonBasherL.setPosition(1);
         }
         if(gamepad2.dpad_down) {
             buttonBasherL.setPosition(0);
         }
         if(gamepad2.dpad_left) {
             buttonBasherR.setPosition(0.8 );
         }
         if(gamepad2.dpad_right) {
             buttonBasherR.setPosition(0.2);
         }
    }
}