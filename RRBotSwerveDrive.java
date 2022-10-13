package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Controls the robot's swerve drive base
 * @author John Brereton
 * @since 2022-10-12
 */

public class RRBotSwerveDrive
{
    RRBotHardware robot;

    private ElapsedTime autoMoveTime = new ElapsedTime();

    private boolean isAutoMove = false;
    private double autoTime;

    /**
     * Constructor gets hardware object from teleop class
     * @param robot contains the hardware elements of the robot
     */
    public RRBotSwerveDrive(RRBotHardware robot)
    {
        this.robot = robot;
    }

    /**
     * Calculates the velocity values for the drive motors in a mecanum configuration.
     * @param leftX X position of left joystick
     * @param leftY Y position of left joystick
     * @param rightX X position of right joystick
     * @param rightY Y position of right joystick
     * @return velocities - array of motor velocities
     */
    public double[] calcVelocities(double leftX, double leftY, double rightX, double rightY)
    {
        double moveX = rightX;
        double moveY1 = leftY;
        double turn = leftX;
        double moveY2 = rightY;

        double v1 = moveY1 + moveX + turn + moveY2;
        double v2 = moveY1 - moveX - turn + moveY2;
        double v3 = moveY1 + moveX - turn + moveY2;
        double v4 = moveY1 - moveX + turn + moveY2;

        double max = Math.abs(v1);
        if(Math.abs(v2) > max)
            max = Math.abs(v2);
        if(Math.abs(v3) > max)
            max = Math.abs(v3);
        if(Math.abs(v4) > max)
            max = Math.abs(v4);
        if(max > 1)
        {
            v1 /= max;
            v2 /= max;
            v3 /= max;
            v4 /= max;
        }

        double[] velocities = {v1, v2, v3, v4};
        return velocities;
    }

    /**
     * Sets the motor power for manual drive. The parameters are sent to calcVelocities.
     * @param leftX X position of left joystick
     * @param leftY Y position of left joystick
     * @param rightX X position of right joystick
     * @param rightY Y position of right joystick
     */
    public void setMotorPower(double leftX, double leftY, double rightX, double rightY)
    {
        //calculate the velocities
        double[] velocities = calcVelocities(leftX, leftY, rightX, rightY);

        //set the motor power
        robot.frontLeftDrive.setPower(velocities[0]);
        robot.frontRightDrive.setPower(velocities[1]);
        robot.rearLeftDrive.setPower(velocities[2]);
        robot.rearRightDrive.setPower(velocities[3]);
    }

    /**
     * Automatically moves the robot based on a set speed and time. Meant to be used in teleop.
     * @param speed speed of movement
     * @param time how long the movement should take
     */
    public void AutoMove(double speed, double time)
    {
        isAutoMove = true;
        autoTime = time;

        autoMoveTime.reset();

        robot.rearRightDrive.setPower(speed);
        robot.rearLeftDrive.setPower(speed);
        robot.frontRightDrive.setPower(speed);
        robot.frontLeftDrive.setPower(speed);
    }

    /**
     * Checks if the movement is done by comparing the time elapsed and the time the movement is set to take.
     */
    public void AutoMoveEndCheck()
    {
        if(autoMoveTime.milliseconds() >= autoTime)
        {
            isAutoMove = false;
        }
    }

    /**
     * Returns whether an auto move is currently occurring
     * @return isAutoMove
     */
    public boolean getIsAutoMove()
    {
        return isAutoMove;
    }
}