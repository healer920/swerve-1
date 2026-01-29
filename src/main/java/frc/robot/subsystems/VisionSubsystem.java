package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import java.util.function.Supplier;


public class VisionSubsystem extends SubsystemBase {
    private final Field2d m_field = new Field2d();
    private final Supplier<Rotation2d> m_gyroYawSupplier;

    /**
     * @param gyroYawSupplier 提供目前的陀螺儀 Yaw 角度 (Rotation2d)
     */
    public VisionSubsystem(Supplier<Rotation2d> gyroYawSupplier) {
        this.m_gyroYawSupplier = gyroYawSupplier;

        SmartDashboard.putData("RobotVisualizer", m_field);
    }

    @Override
    public void periodic() {
        Rotation2d currentYaw = m_gyroYawSupplier.get();

        Pose2d mt2Pose = LimelightHelpers.getBotPose2d_wpiBlue_MegaTag2("limelight", currentYaw);

        if (LimelightHelpers.getTV("limelight")) {
       
            m_field.setRobotPose(mt2Pose); 
        }
    }
}