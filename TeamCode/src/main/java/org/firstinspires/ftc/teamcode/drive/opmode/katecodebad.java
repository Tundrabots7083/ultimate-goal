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
public class katecodebad extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Pose2d myPose = new Pose2d(60,0,Math.toRadians(0));

        drive.setPoseEstimate(myPose);
/*
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(-30, 0), 0)
                .build();

        drive.followTrajectory(traj1);

 */
        drive.followTrajectory(traj(-30, -30, 0));
        drive.followTrajectory(traj(0, 0, 0));
        drive.followTrajectory(traj(30, 30, 0));

        sleep(200);

    }






    
    public Trajectory traj(double x, double y, double endTan){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(x, y), endTan)
                .build();
        return traj;
    }

}
