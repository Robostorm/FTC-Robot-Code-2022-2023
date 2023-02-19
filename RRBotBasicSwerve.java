package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

/**
 * Controls the robot's swerve drive base
 * @author John Brereton
 * @since 2023-01-22
 */

public class RRBotBasicSwerve {
    RRBotHardware robot;

    public RRBotBasicSwerve(RRBotHardware robot) {
        this.robot = robot;
    }

    double driveAngle;
    double driveSpeed;
    double turnSpeed;
    boolean activeTurn = false;

    static final double     COUNTS_PER_MOTOR_REV    = 7;
    static final double     DRIVE_GEAR_REDUCTION    = 6.67/1.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;

    public void swerve(double leftX, double rightX, double rightY) {

        if (rightY < 0) {
            driveAngle = Math.atan2(rightY, rightX)/Math.PI + 1;
            driveSpeed = Math.sqrt(Math.pow(rightX, 2) + Math.pow(rightY, 2));
        } else {
            driveAngle = Math.atan2(rightY, rightX)/Math.PI;
            driveSpeed = -Math.sqrt(Math.pow(rightX, 2) + Math.pow(rightY, 2));
        }
        turnSpeed = leftX;

        // Limit drive speed to 0.5
        driveSpeed /= 2.0;

        if (!activeTurn && driveSpeed >= 0.1 || driveSpeed <= -0.1){
            robot.frontLeftTurn.setPosition(driveAngle);
            robot.frontRightTurn.setPosition(driveAngle);
            robot.rearLeftTurn.setPosition(driveAngle);
            robot.rearRightTurn.setPosition(driveAngle);

            robot.frontLeftDrive.setPower(driveSpeed);
            robot.frontRightDrive.setPower(driveSpeed);
            robot.rearLeftDrive.setPower(driveSpeed);
            robot.rearRightDrive.setPower(driveSpeed);
        } else if (leftX >= 0.1 || leftX <= -0.1) {
            robot.frontLeftTurn.setPosition(0.75);
            robot.frontRightTurn.setPosition(0.25);
            robot.rearLeftTurn.setPosition(0.25);
            robot.rearRightTurn.setPosition(0.75);

            robot.frontLeftDrive.setPower(turnSpeed);
            robot.frontRightDrive.setPower(-turnSpeed);
            robot.rearLeftDrive.setPower(turnSpeed);
            robot.rearRightDrive.setPower(-turnSpeed);

            activeTurn = true;
        } else {
            activeTurn = false;

            robot.frontLeftTurn.setPosition(0.5);
            robot.frontRightTurn.setPosition(0.5);
            robot.rearLeftTurn.setPosition(0.5);
            robot.rearRightTurn.setPosition(0.5);

            robot.frontLeftDrive.setPower(0);
            robot.frontRightDrive.setPower(0);
            robot.rearLeftDrive.setPower(0);
            robot.rearRightDrive.setPower(0);
        }
    }

    public void swerveInches(int inches, double angle)
    {
        robot.frontLeftTurn.setPosition(angle);
        robot.frontRightTurn.setPosition(angle);
        robot.rearLeftTurn.setPosition(angle);
        robot.rearRightTurn.setPosition(angle);

        int goal = robot.frontLeftDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
        while (robot.frontLeftDrive.getCurrentPosition() < goal)
        {
            robot.frontLeftDrive.setPower(DRIVE_SPEED);
            robot.frontRightDrive.setPower(DRIVE_SPEED);
            robot.rearLeftDrive.setPower(DRIVE_SPEED);
            robot.rearRightDrive.setPower(DRIVE_SPEED);
        }

    }
}
