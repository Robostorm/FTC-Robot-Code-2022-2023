package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Defines the robot hardware
 * @author John Brereton
 * @since 2022-10-11
 */

public class RRBotHardware
{
    /* Public OpMode members. */
    public DcMotor frontLeftDrive = null;
    public DcMotor frontRightDrive = null;
    public DcMotor rearLeftDrive = null;
    public DcMotor rearRightDrive = null;
    public CRServo frontLeftTurn = null;
    public CRServo frontRightTurn = null;
    public CRServo rearLeftTurn = null;
    public CRServo rearRightTurn = null;

    public final int ENCODER_TO_ANGLE = 90; // TODO: Set actual value

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public RRBotHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeftDrive  = hwMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hwMap.get(DcMotor.class, "front_right_drive");
        rearLeftDrive = hwMap.get(DcMotor.class, "rear_left_drive");
        rearRightDrive = hwMap.get(DcMotor.class, "rear_right_drive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set motor power on init
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);

        // Set motor run modes
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set motor zero power behavior
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Define and Initialize Servos
        frontLeftTurn = hwMap.get(CRServo.class, "front_left_turn");
        frontRightTurn = hwMap.get(CRServo.class, "front_right_turn");
        rearLeftTurn = hwMap.get(CRServo.class, "rear_left_turn");
        rearRightTurn = hwMap.get(CRServo.class, "rear_right_turn");

        frontLeftTurn.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightTurn.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightTurn.setDirection(DcMotorSimple.Direction.FORWARD);
        rearLeftTurn.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set servo power on init
        frontLeftTurn.setPower(0);
        frontRightTurn.setPower(0);
        rearLeftTurn.setPower(0);
        rearRightTurn.setPower(0);


    }
}