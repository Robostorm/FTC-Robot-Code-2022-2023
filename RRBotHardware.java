package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Defines the robot hardware
 * @author John Brereton, Visvam Rajesh
 * @since 2022-10-11
 */

public class RRBotHardware
{
    /* Public OpMode members. */
    public DcMotor frontLeftDrive = null;
    public DcMotor frontRightDrive = null;
    public DcMotor rearLeftDrive = null;
    public DcMotor rearRightDrive = null;
    public Servo frontLeftTurn = null;
    public Servo frontRightTurn = null;
    public Servo rearLeftTurn = null;
    public Servo rearRightTurn = null;
    public DcMotor armMotor = null;
    public Servo clawServo = null;
/*    public AnalogInput frontLeftEnc = null;
    public AnalogInput frontRightEnc = null;
    public AnalogInput rearLeftEnc = null;
    public AnalogInput rearRightEnc = null;*/

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public RRBotHardware(){}

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeftDrive  = hwMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hwMap.get(DcMotor.class, "front_right_drive");
        rearLeftDrive = hwMap.get(DcMotor.class, "rear_left_drive");
        rearRightDrive = hwMap.get(DcMotor.class, "rear_right_drive");
        armMotor = hwMap.get(DcMotor.class, "arm_motor");

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rearLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set motor power on init
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);
        armMotor.setPower(0);

        // Set motor run modes
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor zero power behavior
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        //Define and Initialize Servos
        frontLeftTurn = hwMap.get(Servo.class, "front_left_turn");
        frontRightTurn = hwMap.get(Servo.class, "front_right_turn");
        rearLeftTurn = hwMap.get(Servo.class, "rear_left_turn");
        rearRightTurn = hwMap.get(Servo.class, "rear_right_turn");
        clawServo = hwMap.get(Servo.class, "claw_servo");

        //frontLeftTurn.setDirection(Servo.Direction.FORWARD);
        //frontRightTurn.setDirection(Servo.Direction.FORWARD);
        //rearRightTurn.setDirection(Servo.Direction.FORWARD);
        //rearLeftTurn.setDirection(Servo.Direction.FORWARD);
        clawServo.setDirection(Servo.Direction.FORWARD);

        // Set servo power on init
        frontLeftTurn.setPosition(0.5);
        frontRightTurn.setPosition(0.5);
        rearLeftTurn.setPosition(0.5);
        rearRightTurn.setPosition(0.5);
        clawServo.setPosition(1);

        /*frontLeftEnc = hwMap.get(AnalogInput.class, "front_left_enc");
        frontRightEnc = hwMap.get(AnalogInput.class, "front_right_enc");
        rearLeftEnc = hwMap.get(AnalogInput.class, "rear_left_enc");
        rearRightEnc = hwMap.get(AnalogInput.class, "rear_right_enc");*/
    }
}