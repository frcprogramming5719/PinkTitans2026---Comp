package frc.robot.subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

/** Add your docs here. */




    public class limelight extends SubsystemBase {
    private final String limelightName;
    String name;
    NetworkTable table;

    public limelight(String name) {
        this.limelightName = name;
        this.table = NetworkTableInstance.getDefault().getTable(name);    

    }





    public double getLimelightTX(){
        return this.table.getEntry("tx").getDouble(0);
    }
    public void setThrottle(int value) {
        this.table.getEntry("throttle_set").setNumber(value);
    }
    public double getThrottle() {
        return this.table.getEntry("throttle_set").getDouble(-1);
    }
    
    public double getLimelightTY(){
        return this.table.getEntry("ty").getDouble(0);
    }
    public double getLimelightTA(){
        return this.table.getEntry("ta").getDouble(0);
    }
    public void lightsOff(){
        NetworkTableInstance.getDefault().getTable(this.name).getEntry("ledMode").setNumber(3); //1
    }
    public void lightsOn(){
        NetworkTableInstance.getDefault().getTable(this.name).getEntry("ledMode").setNumber(3); //0
    }
    public void lightsForceOn(){
        NetworkTableInstance.getDefault().getTable(this.name).getEntry("ledMode").setNumber(3);
    }

        public void lightsForceOnBlink(){
        NetworkTableInstance.getDefault().getTable(this.name).getEntry("ledMode").setNumber(3); //2
    }

    public boolean hasTarget() {
        return table.getEntry("tv").getDouble(0) == 1;
    }
    


    public double getTX() {
        return LimelightHelpers.getTX(limelightName);
    }

    public double getTY() {
        return LimelightHelpers.getTY(limelightName);
    }

    public double getFiducialID() {
        return LimelightHelpers.getFiducialID(limelightName);
    }

    public Pose2d getBotPose2D() {
        return LimelightHelpers.getBotPose2d(limelightName);
    }

    public Pose3d getBotPose3D() {
        return LimelightHelpers.getBotPose3d(limelightName);
    }

    public Pose3d getTargetPoseCameraSpace() {
        return LimelightHelpers.getTargetPose3d_CameraSpace(limelightName);
    }

    public Pose3d getTargetPoseRobotSpace() {
        return LimelightHelpers.getTargetPose3d_RobotSpace(limelightName);
    }

    public double getXOffset() {
        return getTargetPoseRobotSpace().getX();
    }

    public double getYOffset() {
        return getTargetPoseRobotSpace().getY();
    }

    public Rotation2d getRotation() {
        return getBotPose2D().getRotation();
    }
}


    