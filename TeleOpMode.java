package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOpMode", group="Iterative Opmode")

public class TeleOpMode extends OpMode
{
    //flag on stick button status
    private static boolean[] bGamepad1_stat = new boolean[14];
    private static boolean[] bGamepad2_stat = new boolean[14];
    //Main Timer
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    //DcMotors
    private DcMotor leftfront;
    private DcMotor rightfront;
    private DcMotor leftrear;
    private DcMotor rightrear;
    private DcMotor leftChain;
    private DcMotor rightChain;
    private ModernRoboticsI2cGyro gyro;
    //Servos
    private Servo leftServo;
    private Servo rightServo;
    private Servo clawServo;


    @Override
    public void init() {
        //Print
        telemetry.addData("Status", "init() Running");
        telemetry.update();

        //initiating motors (Normal)
        leftfront  = hardwareMap.get(DcMotor.class, "l_f");
        rightfront = hardwareMap.get(DcMotor.class, "r_f");
        leftrear = hardwareMap.get(DcMotor.class, "l_b");
        rightrear = hardwareMap.get(DcMotor.class,"r_b");
        leftChain = hardwareMap.get(DcMotor.class,"leftchain");
        rightChain = hardwareMap.get(DcMotor.class,"rightchain");
        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class,"gyro");

        //initiating Servos (Normal)
        //leftServo = hardwareMap.get(Servo.class,"leftservo");
        //rightServo = hardwareMap.get(Servo.class,"rightservo");
        //clawServo = hardwareMap.get(Servo.class,"clawservo");

        //set motor direction
        leftfront.setDirection(DcMotor.Direction.FORWARD);
        leftrear.setDirection(DcMotor.Direction.FORWARD);
        rightfront.setDirection(DcMotor.Direction.REVERSE);
        rightrear.setDirection(DcMotor.Direction.REVERSE);
        leftChain.setDirection(DcMotor.Direction.FORWARD);
        rightChain.setDirection(DcMotor.Direction.REVERSE);

        //config encoder run modes
        leftChain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightChain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //gyro
        telemetry.addData("Status", "DO NOT MOVE!!!");
        telemetry.update();
        gyro.calibrate();
        telemetry.addData("Status", "Gyro Calibrated");
        telemetry.update();

        //set boolean array to false
        for(int i = 0; i< bGamepad1_stat.length; i++){
            bGamepad1_stat[i]=false;
        }
        for(int i = 0; i< bGamepad2_stat.length; i++){
            bGamepad2_stat[i]=false;
        }

        //Print
        telemetry.addData("Status", "init() Done");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        timer.reset();
    }

    @Override
    public void start() {
        timer.reset();
        telemetry.addData("Timer", "%.3f",timer.time());
        telemetry.update();

        //Config mode on start
        leftChain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightChain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop()   {
        //motor power settings
        double power_1;
        double power_2;
        double power_3;
        double power_4;
        double trim_max;
        //raw stick input (Reverse both Y axis)
        double gamepad1_X = gamepad1.left_stick_x; //leftX
        double gamepad1_Y = -gamepad1.left_stick_y; //leftY
        double gamepad1_Z = gamepad1.right_stick_x; //RightX
        double gamepad1_W = -gamepad1.right_stick_y; //RightY
        //power raw
        power_1 = Functions.MecDrive_RightFront(
                Functions.stickMod(gamepad1_X,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                Functions.stickMod(gamepad1_Y,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                Functions.stickMod(gamepad1_Z,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                Functions.stickMod(gamepad1_W,RoboMap.bDeadzone,RoboMap.bNonLinearInput)
                );
        power_2 = Functions.MecDrive_LeftFront(
                Functions.stickMod(gamepad1_X,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                Functions.stickMod(gamepad1_Y,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                Functions.stickMod(gamepad1_Z,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                Functions.stickMod(gamepad1_W,RoboMap.bDeadzone,RoboMap.bNonLinearInput)
        );
        power_3 = Functions.MecDrive_LeftRear(
                Functions.stickMod(gamepad1_X,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                Functions.stickMod(gamepad1_Y,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                Functions.stickMod(gamepad1_Z,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                Functions.stickMod(gamepad1_W,RoboMap.bDeadzone,RoboMap.bNonLinearInput)
        );
        power_4 = Functions.MecDrive_RightRear(
                Functions.stickMod(gamepad1_X,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                Functions.stickMod(gamepad1_Y,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                Functions.stickMod(gamepad1_Z,RoboMap.bDeadzone,RoboMap.bNonLinearInput),
                Functions.stickMod(gamepad1_W,RoboMap.bDeadzone,RoboMap.bNonLinearInput)
        );
        //scale output so that power is scaled with maximum of 1
        trim_max = Math.max(Math.max(Math.max(Math.abs(power_1),Math.abs(power_2)),
                            Math.max(Math.abs(power_3),Math.abs(power_4))),1);
        power_1 /= trim_max;
        power_2 /= trim_max;
        power_3 /= trim_max;
        power_4 /= trim_max;

        //set motor power
        leftfront.setPower(power_2);
        leftrear.setPower(power_3);
        rightfront.setPower(power_1);
        rightrear.setPower(power_4);

        //set in test mode to disable motors
        /*leftfront.setPower(0);
        leftrear.setPower(0);
        rightfront.setPower(0);
        rightrear.setPower(0);*/

        //chain mapped on button A on gamepad1
        double dChainSpeed = 0.25; //slower for testing
        if(gamepad1.a&&!bGamepad1_stat[0]){
            //press
            bGamepad1_stat[0]=true;
            leftChain.setPower(dChainSpeed);
            rightChain.setPower(dChainSpeed);
        }else if(gamepad1.a){
            //hold
            leftChain.setPower(dChainSpeed);
            rightChain.setPower(dChainSpeed);
        }else{
            //release OR nothing
            bGamepad1_stat[0]=false;
            leftChain.setPower(0);
            rightChain.setPower(0);
        }




        //put data into dashboard
        telemetry.addData("Config_Main_Timer", "Run Time: " + timer.toString());
        telemetry.addData("Config_trim",trim_max);
         //telemetry.addData("gamepad1.atRest()",gamepad1.atRest());
        telemetry.addData("Config_gamepad1.Input","LX (%.2f), LY (%.2f), RX (%.2f), RY (%.2f)",
                gamepad1_X,gamepad1_Y,gamepad1_Z,gamepad1_W);
        telemetry.addData("Config_MotorPower","1 (%.2f), 2 (%.2f), 3 (%.2f),4 (%.2f)",
                power_1,power_2,power_3,power_4);

    }

    @Override
    public void stop() {
        leftfront.setPower(0);
        leftrear.setPower(0);
        rightfront.setPower(0);
        rightrear.setPower(0);
        leftChain.setPower(0);
        rightChain.setPower(0);
        telemetry.clearAll();
        telemetry.addData("Status","Stop Enforced");
        telemetry.update();
    }

}

/* Buttons
 * //when held
        if(gamepad1.a&&!bGamepad1_stat[0]){
            //press
            bGamepad1_stat[0]=true;
            testMotor.setPower(0.5);
        }else if(gamepad1.a){
            //hold
            testMotor.setPower(0.5);
        }else{
            //release OR nothing
            bGamepad1_stat[0]=false;
            testMotor.setPower(0);
        }

        //when press and repress
        if(gamepad1.a&&!bGamepad1_stat[0]){
            //Moment of Pressing the Button
            bGamepad1_stat[0] = true;
            if(testMotor.getPower()!=0){
                //if it is the second time pressing
                testMotor.setPower(0);
            }else{
                //if it is the first time pressing
                testMotor.setPower(0.5);
            }
        }else if(!gamepad1.a){
            //reset gamepad state
            bGamepad1_stat[0] = false;
        }else{}//when motor is running and button is pressed (ignore it)
* */