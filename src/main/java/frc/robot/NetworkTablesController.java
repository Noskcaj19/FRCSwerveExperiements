package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

public class NetworkTablesController extends GenericHID {
    public NetworkTablesController(int port) {
        super(port);
    }

    public double getLeftY() {
        return getRawAxis(1);
        // return getRawAxis(XboxController.Axis.kLeftY.value);
    }

    public double getLeftX() {
        return getRawAxis(0);
        // return getRawAxis(XboxController.Axis.kLeftX.value);
    }

    public double getRightY() {
        return getRawAxis(4);
        // return getRawAxis(XboxController.Axis.kRightY.value);
    }

    public double getRightX() {
        return getRawAxis(3);
        // return getRawAxis(XboxController.Axis.kRightX.value);
    }

    public boolean getAButton() {
        return getRawButton(XboxController.Button.kA.value);
    }

    @Override
    public int getAxisCount() {
        return 6;
    }

    @Override
    public int getAxisType(int axis) {
        return 0;
    }

    @Override
    public int getButtonCount() {
        return 14;
    }

    @Override
    public double getRawAxis(int axis) {
        return NetworkTableInstance.getDefault().getEntry("/RemoteControl/"+getPort()+"/Axis/"+axis).getDouble(0);
    }

    @Override
    public boolean getRawButton(int button) {
        return NetworkTableInstance.getDefault().getEntry("/RemoteControl/" + getPort()+"/Button/"+button).getBoolean(false);
    }

    @Override
    public boolean getRawButtonPressed(int button) {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getRawButtonReleased(int button) {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public String getName() {
        return "NetworkTables";
    }

    @Override
    public int getPOV() {
        return getPOV(0);
    }

    @Override
    public int getPOV(int pov) {
        return -1;
    }

    @Override
    public int getPOVCount() {
        return 0;
    }

    @Override
    public HIDType getType() {
        return HIDType.kHIDGamepad;
    }

    @Override
    public boolean isConnected() {
        // TODO:!!!
        return false;
    }

    @Override
    public void setOutput(int outputNumber, boolean value) {
    }

    @Override
    public void setOutputs(int value) {
    }

    @Override
    public void setRumble(RumbleType type, double value) {
    }

    public boolean getRightBumper() {
        return getRawButton(XboxController.Button.kRightBumper.value);
    }

}
