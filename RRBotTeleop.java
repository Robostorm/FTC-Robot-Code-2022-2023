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

        // Show the elapsed game time and wheel power.
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    /*
     * Updates the drive system with manual and automatic movements
     */
    public void DriveUpdate(){

        telemetry.addData("Gamepad(Left)", "X: (%.2f), Y: (%.2f)", gamepad1.left_stick_x, gamepad1.left_stick_y);
        telemetry.addData("Gamepad(Right)", "X: (%.2f), Y: (%.2f)", gamepad1.right_stick_x, gamepad1.right_stick_y);
        telemetry.addData("Encoder", "FL: (%.2f) FR: (%.2f) RL: (%.2f) RR: (%.2f)", robot.frontLeftEnc.getVoltage(), robot.frontRightEnc.getVoltage(), robot.rearRightEnc.getVoltage(), robot.rearLeftEnc.getVoltage());
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
}
