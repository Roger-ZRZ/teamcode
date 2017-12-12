package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@Autonomous(name="AutonomousMode", group="Linear Opmode")
public class AutonomousMode extends LinearOpMode {

    //Main Timer
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    //DcMotors
    private DcMotor leftfront;
    private DcMotor rightfront;
    private DcMotor leftrear;
    private DcMotor rightrear;
    private DcMotor leftChain;
    private DcMotor rightChain;
    private DcMotor arm_1;
    private DcMotor arm_2;

    //Servos
    private Servo f_clawServo;
    private Servo b_clawServo;
    private Servo arm_servo_1;
    private Servo arm_servo_2;


    //wheel_servos
    private CRServo right_wheel;
    private CRServo left_wheel;

    private ColorSensor colorSensor;

    public int Chain_exp = 0;

    @Override
    public void runOpMode() {
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
        arm_1 = hardwareMap.get(DcMotor.class,"arm_1");
        arm_2 = hardwareMap.get(DcMotor.class,"arm_2");

        //initiating Servos (Normal)
        f_clawServo = hardwareMap.get(Servo.class,"fclaw");
        b_clawServo = hardwareMap.get(Servo.class,"bclaw");
        arm_servo_1 = hardwareMap.get(Servo.class,"armservo1");
        arm_servo_2 = hardwareMap.get(Servo.class,"armservo2");


        //init CRServos
        right_wheel = hardwareMap.get(CRServo.class,"rwheel");
        left_wheel = hardwareMap.get(CRServo.class,"lwheel");

        //init color sensor
        colorSensor = hardwareMap.get(ColorSensor.class,"color");

        //set motor direction
        leftfront.setDirection(DcMotor.Direction.FORWARD);
        leftrear.setDirection(DcMotor.Direction.FORWARD);
        rightfront.setDirection(DcMotor.Direction.REVERSE);
        rightrear.setDirection(DcMotor.Direction.REVERSE);
        leftChain.setDirection(DcMotor.Direction.FORWARD);
        rightChain.setDirection(DcMotor.Direction.REVERSE);
        right_wheel.setDirection(CRServo.Direction.FORWARD);
        left_wheel.setDirection(CRServo.Direction.REVERSE);
        arm_1.setDirection(CRServo.Direction.FORWARD);
        arm_2.setDirection(CRServo.Direction.REVERSE);


        //config encoder run modes
        leftChain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightChain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightrear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftrear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Config mode on start
        leftChain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightChain.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightrear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftrear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        f_clawServo.scaleRange(0.3,0.9);
        b_clawServo.scaleRange(0.4,0.9);
        telemetry.addData("F_claw POS",f_clawServo.getPosition());
        telemetry.addData("B_claw POS",b_clawServo.getPosition());

        right_wheel.setPower(0);
        left_wheel.setPower(0);

        //Print
        telemetry.addData("Status", "init() Done");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            int iColor = 0;
            double dRed = colorSensor.red();
            double dBlue = colorSensor.blue();
            //=1 red //=0 blue //=-1 nothing
            if(dRed>dBlue){
                iColor = 1;
                telemetry.addData("Color is Red",iColor);
            }else if(dRed<dBlue){
                iColor = -1;
                telemetry.addData("Color is Blue",iColor);
            }else{
                iColor = 0;
                telemetry.addData("No Color",iColor);
            }
            /*
            rightfront.setPower(0.5);
            rightrear.setPower(0.5);
            leftfront.setPower(0.5);
            leftrear.setPower(0.5);

            rightfront.setTargetPosition(5000);
            rightrear.setTargetPosition(5000);
            leftfront.setTargetPosition(5000);
            leftrear.setTargetPosition(5000);
            */


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + timer.toString());
            telemetry.addData("Motors", "left (%d), right (%d)", leftfront.getCurrentPosition(), rightfront.getCurrentPosition());
            telemetry.update();
        }
    }
}
