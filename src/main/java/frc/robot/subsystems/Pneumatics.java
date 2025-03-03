package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;

public class Pneumatics extends SubsystemBase{
    private Compressor mCompressor;

    public Pneumatics() {
        this.mCompressor = new Compressor(Constants.kPCMCANId, PneumaticsModuleType.CTREPCM);
    }

    public Command Init() {
        return this.runOnce(() -> {
            this.mCompressor.enableDigital();
        });
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Compressor", this.mCompressor.isEnabled());
    }
}
