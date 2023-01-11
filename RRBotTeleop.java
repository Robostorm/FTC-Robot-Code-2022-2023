package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="Swerve Drive", group="Iterative Opmode")

public class RRBotTeleop extends OpMode {
    // Declare OpMode members.
    RRBotHardware robot = new RRBotHardware();
    RRBotSwerveDrive swerve = new RRBotSwerveDrive(robot);
    private ElapsedTime runtime = new ElapsedTime();

    int armPosition = 0;
    double lastArmMove = (double) runtime.time();
    boolean isClawOpen = false;

    // Construct Swerve Drive Class
    //RRBotSwerveDrive drive = new RRBotSwerveDrive(robot);

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        DriveUpdate();

        ArmUpdate();

        // Show the elapsed game time and wheel power.
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.clawServo.setPosition(1);
    }

    /*
     * Updates the drive system with manual and automatic movements.
     */
    public void DriveUpdate(){

        telemetry.addData("Gamepad(Left)", "X: (%.2f), Y: (%.2f)", gamepad1.left_stick_x, gamepad1.left_stick_y);
        telemetry.addData("Gamepad(Right)", "X: (%.2f), Y: (%.2f)", gamepad1.right_stick_x, gamepad1.right_stick_y);
        telemetry.addData("Encoder", "FL: (%.2f) FR: (%.2f) RL: (%.2f) RR: (%.2f)", robot.frontLeftEnc.getVoltage() * robot.ENCODER_TO_ANGLE, robot.frontRightEnc.getVoltage() * robot.ENCODER_TO_ANGLE, robot.rearRightEnc.getVoltage() * robot.ENCODER_TO_ANGLE, robot.rearLeftEnc.getVoltage() * robot.ENCODER_TO_ANGLE);
        telemetry.addData("Last Error", swerve.getLastError());
        /*if(!drive.getIsAutoMove()) {
            drive.setMotorPower(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, -gamepad1.right_stick_y, true);
        } else{
            drive.AutoMoveEndCheck();
        }*/
        if(!swerve.getIsAutoMove()){
            if(gamepad1.left_stick_x > 0.1f && -gamepad1.left_stick_y > 0.1f)
            {
                swerve.setMotorPower(gamepad1.left_stick_x, -gamepad1.left_stick_y);
                telemetry.addData("Update", "Passed Motor Power Check");
                swerve.setServoAngle(gamepad1.left_stick_x, -gamepad1.left_stick_y);
                telemetry.addData("Update", "Passed Servo Angle Check");
            }
            if(gamepad1.right_stick_x > 0.1f && -gamepad1.right_stick_y > 0.1f)
                swerve.TurnFacing(gamepad1.right_stick_x, -gamepad1.right_stick_y);
            telemetry.addData("Update", "Passed Turn Facing Check");
        }else{
            swerve.AutoMoveEndCheck();
        }
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
