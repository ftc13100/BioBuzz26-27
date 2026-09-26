package org.firstinspires.ftc.teamcode


import com.pedropathing.api.Paths.*
import com.pedropathing.api.PoseFactory
import com.pedropathing.follower.Follower
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.Scheduler.schedule
import com.pedropathing.ivy.commands.Commands.*
import com.pedropathing.ivy.groups.Groups.sequential
import com.pedropathing.ivy.pedro.PedroCommands.follow
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.pedropathing.paths.interpolator.Interpolator
import org.firstinspires.ftc.teamcode.pedro.Constants


@Autonomous(name = "Auto", group = "Autonomous")
class Auto : LinearOpMode() {


    private lateinit var follower: Follower


    private val poseFactory = PoseFactory.degrees()


    private val start = poseFactory.of(56.0, 8.0, 90.0)
    private val path1 = poseFactory.of(9.1511, 7.6278, -90.0)
    private val path1Control1 = poseFactory.of(3.0086, 30.8599, 0.0)
    private val path1Segment1Start = poseFactory.of(9.1511, 7.6278, -180.0)
    private val path1Segment1End = poseFactory.of(9.1511, 7.6278, -90.0)
    private val path1Segment2Heading = poseFactory.of(9.1511, 7.6278, -90.0)
    private val point2 = poseFactory.of(47.4276, 131.437, -270.0)
    private val point2Segment1Start = poseFactory.of(47.4276, 131.437, -90.0)
    private val point2Segment1End = poseFactory.of(47.4276, 131.437, -109.0)
    private val point2Segment3Start = poseFactory.of(47.4276, 131.437, -107.0)
    private val point2Segment3End = poseFactory.of(47.4276, 131.437, 90.0)
    private val point3 = poseFactory.of(47.0, 23.1247, 150.0)
    private val point3Control1 = poseFactory.of(18.6468, 64.6482, 0.0)
    private val point3Segment1Start = poseFactory.of(47.0, 23.1247, 90.0)
    private val point3Segment1End = poseFactory.of(47.0, 23.1247, 72.0)
    private val point3Segment3Start = poseFactory.of(47.0, 23.1247, 98.0)
    private val point3Segment3End = poseFactory.of(47.0, 23.1247, 150.0)
    private val point4 = poseFactory.of(10.1902, 45.9317, 180.0)
    private val point4Segment2Start = poseFactory.of(10.1902, 45.9317, 150.0)
    private val point4Segment2End = poseFactory.of(10.1902, 45.9317, -180.0)
    private val point5 = poseFactory.of(50.755, 27.0263, -270.0)
    private val point5Segment1Heading = poseFactory.of(50.755, 27.0263, -180.0)
    private val point5Segment2Start = poseFactory.of(50.755, 27.0263, -180.0)
    private val point5Segment2End = poseFactory.of(50.755, 27.0263, 90.0)
    private val point6 = poseFactory.of(7.4412, 93.3075, 115.2748)
    private val point6Control1 = poseFactory.of(27.53, 50.6325, 0.0)


    // Autonomous routine
    fun autoRoutine(): Command = sequential(
        follow(follower, path1()),
        follow(follower, path2()),
        follow(follower, path3()),
        follow(follower, path4()),
        follow(follower, path5()),
        follow(follower, path6())
    )


    override fun runOpMode() {
        Scheduler.reset()
        follower = Constants.create(hardwareMap)
        follower.setPose(start)
        follower.update()


        waitForStart()
        schedule(autoRoutine())


        while (opModeIsActive()) {
            follower.update()
            Scheduler.execute()


            telemetry.addData("x", follower.pose().x())
            telemetry.addData("y", follower.pose().y())
            telemetry.addData("heading", follower.pose().heading())


            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint())
                telemetry.addData("Path number", follower.pathIndex())
            }


            telemetry.update()
        }
    }


    fun path1(): Path = curve(start, path1Control1, path1).heading(Interpolator.piecewise().until(0.6855, Interpolator.linear(path1Segment1Start, path1Segment1End)).until(1.0, Interpolator.constant(path1Segment2Heading)))


    fun path2(): Path = line(path1, point2).heading(Interpolator.piecewise().until(0.1463, Interpolator.linear(point2Segment1Start, point2Segment1End)).until(0.7434, Interpolator.tangent.reverse()).until(1.0, Interpolator.linear(point2Segment3Start, point2Segment3End)))


    fun path3(): Path = curve(point2, point3Control1, point3).heading(Interpolator.piecewise().until(0.1518, Interpolator.linear(point3Segment1Start, point3Segment1End)).until(0.6437, Interpolator.tangent.reverse()).until(1.0, Interpolator.linear(point3Segment3Start, point3Segment3End)))


    fun path4(): Path = line(point3, point4).heading(Interpolator.piecewise().until(0.7633, Interpolator.tangent).until(1.0, Interpolator.linear(point4Segment2Start, point4Segment2End)))


    fun path5(): Path = line(point4, point5).heading(Interpolator.piecewise().until(0.2269, Interpolator.constant(point5Segment1Heading)).until(1.0, Interpolator.linear(point5Segment2Start, point5Segment2End)))


    fun path6(): Path = curve(point5, point6Control1, point6).tangent()
}







































































