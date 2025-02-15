package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BeamBreak extends SubsystemBase {
    DigitalInput beam;

    public BeamBreak() {
        beam = new DigitalInput(0);
    }

    @Override
    public void periodic() {
        boolean out = beam.get();
        int portHandle = beam.getPortHandleForRouting();
        int channel = beam.getChannel();
        SmartDashboard.putBoolean("Beam ", out);
        SmartDashboard.putBoolean("isAnalogTrigger", beam.isAnalogTrigger());

    }

}