package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.LimelightHelpers;
import com.ctre.phoenix6.hardware.Pigeon2;
//import com.ctre.phoenix6.swerve.SwerveModule;

public class DriveSubsystem extends SubsystemBase {

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(0.32, 0.385), // 前左
        new Translation2d(0.32, -0.385),  // 前右
        new Translation2d(-0.32, 0.385),  // 後左
        new Translation2d(-0.32, -0.385)    // 後右
    );

    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final Field2d m_field = new Field2d();
    private final Pigeon2 m_gyro = new Pigeon2(11);
    
    
    public DriveSubsystem() {

        // 初始化 PoseEstimator
        m_poseEstimator = new SwerveDrivePoseEstimator(
                kDriveKinematics,
                getGyroRotation(),
                getModulePositions(),
                new Pose2d());


        SmartDashboard.putData("Field", m_field);
    }

    @Override
    public void periodic() {
        // 更新 Odometry (底盤編碼器部分)
        m_poseEstimator.update(getGyroRotation(), getModulePositions());

        // 取得目前的陀螺儀 Yaw 角度
        //Rotation2d yaw = getGyroRotation();

        // 從 LimelightHelpers 取得 MT2 結果
        LimelightHelpers.PoseEstimate mt2Estimate = LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2("limelight", getGyroRotation());

        // 檢查是否有看到 Target 且數據可信
        if (mt2Estimate != null && mt2Estimate.tagCount > 0) {
            // 加入視覺測量值到 PoseEstimator
            if (mt2Estimate.avgTagDist < 4.0) {
                m_poseEstimator.addVisionMeasurement(
                        mt2Estimate.pose,
                        mt2Estimate.timestampSeconds);
            }
        }

        //從 PoseEstimator 取得融合後的最終結果
        m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());

    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            new SwerveModulePosition(0, new Rotation2d()),
            new SwerveModulePosition(0, new Rotation2d()),
            new SwerveModulePosition(0, new Rotation2d()),
            new SwerveModulePosition(0, new Rotation2d())
        };
    }

        // 輔助方法：獲取陀螺儀 Rotation2d
        public Rotation2d getGyroRotation() {
            return m_gyro.getRotation2d();
        }

        public Pose2d getPose(){
            return m_poseEstimator.getEstimatedPosition();
        }
}


