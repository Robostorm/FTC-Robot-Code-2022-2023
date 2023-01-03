package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Swerve Drive", group="Iterative Opmode")
@Disabled
public class RRBotTeleop extends OpMode {
    // Declare OpMode members.
    RRBotHardware robot = new RRBotHardware();
    RRBotSwerveDrive swerve = new RRBotSwerveDrive(robot);
    private ElapsedTime runtime = new ElapsedTime();
    double turnAngle = 0.0;

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
        DriveUpdate();

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
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
        /*if(!drive.getIsAutoMove()) {
            drive.setMotorPower(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, -gamepad1.right_stick_y, true);
        } else{
            drive.AutoMoveEndCheck();
        }*/
        swerve.setMotorPower(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        swerve.setServoAngle(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        swerve.TurnFacing(gamepad1.right_stick_x, -gamepad1.right_stick_y);
    }
}
