// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LED extends SubsystemBase {
    private static SerialPort serialPort = new SerialPort(115200, Port.kMXP);
    private static COLORS currentColor = null;

    public class Message {
        public final int start, end;
        public final COLORS color;

        public Message(int start, int end, COLORS color) {
            this.color = color;
            this.start = start;
            this.end = end;
        }
    }

    public LED() {
        setLEDColor(new Message(0, Constants.LED_LENGTH, COLORS.OFF));
        //setDefaultCommand(new RunCommand(() -> setLEDColor(new Message(0, Constants.LED_LENGTH, getColorState())), this));
    }

    @Override
    public void periodic() {
        
        //setLEDColor(new Message(0, Constants.LED_LENGTH, getColorState()));
    }

    /**Periodic function for obtaining the current color based on various robot statuses
     * @return The current color, with a descending priority as listed in the function
    */
    public COLORS getColorState() {
        return RobotContainer.isBlue() ? COLORS.BLUE : COLORS.RED;
    }

    public static void setLEDColor(Message message){
        System.out.println("sending led");
        //if (message.color == currentColor) return;
        currentColor = message.color;

        //Construct
        byte[] data = new byte[7];
        data[0] = (byte)(message.start);
        data[1] = (byte)(message.start >> 8);
        data[2] = (byte)(message.end);
        data[3] = (byte)(message.end >> 8);
        data[4] = message.color.r;
        data[5] = message.color.g;
        data[6] = message.color.b;
        if (serialPort.write(data, data.length) < data.length) System.out.println("Failed to send (" + data.length + ") bytes over serial");
    }

    public Command setLedCommand(Message message) {
        return new InstantCommand(() -> setLEDColor(message));
    }

    public enum COLORS{
        RED(255, 0, 0),
        BLUE(0, 0, 255),
        GREEN(0, 255, 0),
        ORANGE(255, 20, 0),
        YELLOW(255, 80, 0),
        PURPLE(60,0,255),
        AQUA(0,255,255),
        HOTPINK(255,0,10),
        OFF(0,0,0);

        public byte r;
        public byte g;
        public byte b;
        private COLORS(int _r, int _g, int _b) {
            r = (byte)_r;
            g = (byte)_g;
            b = (byte)_b;
        }
    }
}