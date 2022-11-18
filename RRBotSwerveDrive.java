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
    private int encoderPos = 0; //TODO: Get Encoder Position

    /**
     * Constructor gets hardware object from teleop class
     * @param robot contains the hardware elements of the robot
     */
    public RRBotSwerveDrive(RRBotHardware robot)
    {
        this.robot = robot;
    }

    /**
     * Calculates the power of the motors for a swerve drive
     * @param leftX X position of the left joystick
     * @param leftY Y position of the left joystick
     */
    public double getPower(double leftX, double leftY)
    {
        //The drive power is the hypotenuse of the right triangle formed by leftX and leftY
        return Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2));
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
        //double[] velocities = calcVelocities(leftX, leftY, rightX, rightY);
        double power = getPower(leftX, leftY);

        //set the motor power
        robot.frontLeftDrive.setPower(power);
        robot.frontRightDrive.setPower(power);
        robot.rearLeftDrive.setPower(power);
        robot.rearRightDrive.setPower(power);
    }

    /**
     * Calculates the angle(in degrees) of the servos in a standard swerve drive
     * @param leftX X position of left joystick
     * @param leftY Y position of left joystick
     */
    public double getAngle(double leftX, double leftY)
    {
        //The angle is the inverse tangent of the right triangle formed by leftX and leftY
        double radians = Math.atan(leftY / leftX);

        return (radians * 180) / Math.PI;
    }

    public void setServoAngle(double leftX, double leftY)
    {
        double angle = getAngle(leftX, leftY);

        //TODO: Write code to set servo position based on an angle in degrees
        int currentPos = 0; //TODO: Get encoder position

        if(angle < 0)
        {
            if((angle * robot.ENCODER_TO_ANGLE) > (encoderPos * -1))
            {
                robot.frontRightTurn.setPower(-0.5);
                robot.frontLeftTurn.setPower(-0.5);
                robot.rearRightTurn.setPower(-0.5);
                robot.rearLeftTurn.setPower(-0.5);
            }else if((angle * robot.ENCODER_TO_ANGLE) < (encoderPos * -1))
            {
                robot.frontRightTurn.setPower(0);
                robot.frontLeftTurn.setPower(0);
                robot.rearRightTurn.setPower(0);
                robot.rearLeftTurn.setPower(0);
            }else
            {
                robot.frontRightTurn.setPower(0);
                robot.frontLeftTurn.setPower(0);
                robot.rearRightTurn.setPower(0);
                robot.rearLeftTurn.setPower(0);
            }
        }else if(angle > 0)
        {
            if((angle * robot.ENCODER_TO_ANGLE) > encoderPos)
            {
                robot.frontRightTurn.setPower(-0.5);
                robot.frontLeftTurn.setPower(-0.5);
                robot.rearRightTurn.setPower(-0.5);
                robot.rearLeftTurn.setPower(-0.5);
            }else if((angle * robot.ENCODER_TO_ANGLE) < encoderPos)
            {
                robot.frontRightTurn.setPower(0);
                robot.frontLeftTurn.setPower(0);
                robot.rearRightTurn.setPower(0);
                robot.rearLeftTurn.setPower(0);
            }else
            {
                robot.frontRightTurn.setPower(0);
                robot.frontLeftTurn.setPower(0);
                robot.rearRightTurn.setPower(0);
                robot.rearLeftTurn.setPower(0);
            }
        }

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