package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Tank Drive", group="Linear Opmode")

public class RRBotTankDrive extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    int armPosition = 0;
    double lastArmMove = (double) runtime.time();
    boolean isClawOpen = false;
    private RRBotHardware robot ;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot = new RRBotHardware();
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            leftPower  = -gamepad1.left_stick_y ;
            rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            robot.frontLeftDrive.setPower(leftPower * 1.5);
            robot.frontRightDrive.setPower(rightPower * 1.5);
            robot.rearLeftDrive.setPower(leftPower * 1.5);
            robot.rearRightDrive.setPower(rightPower * 1.5);

            ArmUpdate();
            telemetry.addData("Arm Pos", armPosition);

//            if(robot.frontLeftEnc.getVoltage()  * robot.ENCODER_TO_ANGLE > 123.07)
//                robot.frontLeftTurn.setPower(0.2);
//            if(robot.frontLeftEnc.getVoltage() * robot.ENCODER_TO_ANGLE < 123.07)
//                robot.frontLeftTurn.setPower(-0.2);
//            else
//                robot.frontLeftTurn.setPower(0);
//            if(robot.frontRightEnc.getVoltage()  * robot.ENCODER_TO_ANGLE > 193.32)
//                robot.frontRightTurn.setPower(0.2);
//            if(robot.frontRightEnc.getVoltage() * robot.ENCODER_TO_ANGLE < 193.32)
//                robot.frontRightTurn.setPower(-0.2);
//            else
//                robot.frontRightTurn.setPower(0);
//            if(robot.rearRightEnc.getVoltage()  * robot.ENCODER_TO_ANGLE > 43.5)
//                robot.rearRightTurn.setPower(0.2);
//            if(robot.rearRightEnc.getVoltage() * robot.ENCODER_TO_ANGLE < 43.5)
//                robot.rearRightTurn.setPower(-0.2);
//            else
//                robot.rearRightTurn.setPower(0);
//            if(robot.rearLeftEnc.getVoltage()  * robot.ENCODER_TO_ANGLE > 182.72)
//                robot.rearLeftTurn.setPower(0.2);
//            if(robot.rearLeftEnc.getVoltage() * robot.ENCODER_TO_ANGLE < 182.72)
//                robot.rearLeftTurn.setPower(-0.2);
//            else
//                robot.rearLeftTurn.setPower(0);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("REAR RIGHT ENCODER", robot.frontLeftEnc.getVoltage()  * robot.ENCODER_TO_ANGLE);
            telemetry.update();
        }

        robot.clawServo.setPosition(0);
    }

    /**
     * Arm Positions:
     * 0 - Lowest position, used when picking up freight
     * 1 - Used when placing freight in shared shipping hub or lowest level or blue or red shipping hubs
     * 2 - Used when placing freight in the 2nd level of the blue or red shipping hubs
     * 3 - Used when placing freight in the 3rd and highest level of the blue or red shipping hubs
     * 4 - Arm folds to fit within 18x18x18 sizing constrains
     */
    // Sets moves the arm between 4 preset positions
    public void ArmUpdate(){
        // Increases the armPosition by one every time dpad up is pressed
        if (gamepad1.dpad_up && armPosition < 5 && runtime.time() - lastArmMove > 0.5) {
            armPosition += 1;
            lastArmMove = (double) runtime.time();
        }
        // Decreases the armPosition variable by one every time dpad down is pressed
        else if (gamepad1.dpad_down && armPosition > 0 && runtime.time() - lastArmMove > 0.5) {
            armPosition -= 1;
            lastArmMove = (double) runtime.time();
        }

        // Sets the position of the arm to the position set in the armPosition variable
        robot.armMotor.setPower(1);
        if(armPosition==0){
            robot.armMotor.setTargetPosition(0);
        }else if(armPosition==1){
            robot.armMotor.setTargetPosition(700);
        }else if(armPosition==2){
            robot.armMotor.setTargetPosition(1400);
        }else if(armPosition==3){
            robot.armMotor.setTargetPosition(2100);
        }else if(armPosition==4){
            robot.armMotor.setTargetPosition(2800);
        }else if(armPosition==5){
            robot.armMotor.setTargetPosition(3300);
        }
        if(gamepad1.a)
        {
            if(isClawOpen)
            {
                robot.clawServo.setPosition(1);
                isClawOpen = false;
            }else{
                robot.clawServo.setPosition(0);
                isClawOpen = true;
            }
        }
    }
}
