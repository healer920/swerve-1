package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveModule{

    private static final double kDriveEncoderRotationsToMeters = 0.0254 * Math.PI/6.75 ;
    
    private final TalonFX m_driveMotor;
    private final CANcoder m_turningEncoder;
        
            public SwerveModule(int driveID, int turningID, int encoderID) {
                m_driveMotor = new TalonFX(driveID);
                m_turningEncoder = new CANcoder(encoderID);
            //TODO Auto-generated constructor stub
            }

            public SwerveModulePosition getPosition() {
                // 1. 驅動馬達移動的距離 (公尺)
                double distance = m_driveMotor.getPosition().getValueAsDouble() * kDriveEncoderRotationsToMeters;
    
                // 2. 轉向角度 (Rotation2d)
                Rotation2d angle = Rotation2d.fromRotations(m_turningEncoder.getAbsolutePosition().getValueAsDouble());

                // 3. 兩個數值打包成一個 Position 物件回傳
                return new SwerveModulePosition(distance, angle);
            }
}

   
