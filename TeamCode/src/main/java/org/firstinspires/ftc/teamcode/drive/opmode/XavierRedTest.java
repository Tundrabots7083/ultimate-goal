package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.canvas.Spline;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
/*
 * This is an example of a mechanical person trying to code.
 */


@Autonomous(group = "drive")
public class XavierRedTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        // X and Y are reversed.

        Pose2d myPose = new Pose2d(-50,40,Math.toRadians(0));

        drive.setPoseEstimate(myPose);
/*
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(-30, 0), 0)
                .build();

        drive.followTrajectory(traj1);

 */

        /*drive.followTrajectory(traj(50, 40, 0));
        sleep(100);
        drive.followTrajectory(traj(0, -40, 0));
        sleep(100);
        drive.followTrajectory(traj(50, 40, 0));
        sleep(100);*/

        drive.followTrajectory(traj(90, -30, 0));

    }







    public Trajectory traj(double x, double y, double endTan){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(x, y), endTan)
                .build();
        return traj;
    }

    public Trajectory lineTo(double x, double y){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(x, y))
                .build();
        return traj;
    }
}