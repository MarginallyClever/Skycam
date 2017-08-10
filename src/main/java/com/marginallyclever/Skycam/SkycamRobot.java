package com.marginallyclever.Skycam;


import java.awt.Color;
import java.awt.Component;
import java.util.ArrayList;
import java.util.prefs.Preferences;

import javax.swing.JMenu;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;

import com.marginallyclever.communications.NetworkConnection;
import com.marginallyclever.communications.NetworkConnectionListener;
import com.marginallyclever.communications.NetworkConnectionManager;

public class SkycamRobot
implements NetworkConnectionListener {
	private static String CUE="> ";
	private static String NEWLINE="\n";
	
	public NetworkConnection serialPort;
	public boolean portOpened=false;
	public boolean portConfirmed=false;
	public String portName;
	
	// settings
	private Preferences prefs;
	
	// menus & GUIs
	JTextArea log = new JTextArea();
	JScrollPane logPane;
    
    // communications
    String line3;
    ArrayList<String> commandQueue = new ArrayList<String>();

    // Listeners which should be notified of a change to the percentage.
    private ArrayList<SerialConnectionReadyListener> listeners = new ArrayList<SerialConnectionReadyListener>();

	
	public SkycamRobot(String name) {
		prefs = Preferences.userRoot().node("SkycamRobot");
	}
	
	public void Log(String msg) {
		log.append(msg);
		log.setCaretPosition(log.getText().length());
	}
	
	
	public boolean confirmPort() {
		if(!portOpened) return false; 
		if(portConfirmed) return true;
		
		String hello = "HELLO WORLD! I AM SKYCAM #";
		int found=line3.lastIndexOf(hello);
		if(found >= 0) {
			/*
			// get the UID reported by the robot
			String[] lines = line3.substring(found+hello.length()).split("\\r?\\n");
			if(lines.length>0) {
				try {
					robot_uid = Long.parseLong(lines[0]);
				}
				catch(NumberFormatException e) {}
			}
			
			// new robots have UID=0
			if(robot_uid==0) {
				// Try to set a new one
				GetNewRobotUID();
			}
			mainframe.setTitle("Drawbot #"+Long.toString(robot_uid));

			// load machine specific config
			LoadConfig();
			if(limit_top==0 && limit_bottom==0 && limit_left==0 && limit_right==0) {
				UpdateConfig();
			} else {
				SendConfig();
			}
			previewPane.setMachineLimits(limit_top, limit_bottom, limit_left, limit_right);
			
			// load last known paper for this machine
			GetRecentPaperSize();
			if(paper_top==0 && paper_bottom==0 && paper_left==0 && paper_right==0) {
				UpdatePaper();
			}
*/
			portConfirmed=true;
		}
		return portConfirmed;
	}
	
	
	@Override
	public void dataAvailable(NetworkConnection arg0,String data) {
		if(!portOpened) return;
		Log(data);
		line3+=data;
		// wait for the cue to send another command
		if(line3.lastIndexOf(CUE)!=-1) {
			if(confirmPort()) {
				line3="";
				sendQueuedCommand();
			}
		}
	}
	
	
	protected void sendQueuedCommand() {
		if(!portOpened) return;
		
		if(commandQueue.size()==0) {
		      notifyListeners();
		      return;
		}
		String command=commandQueue.remove(0)+";"+NEWLINE;
		Log(command);
		try {
			serialPort.sendMessage(command);
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	
	public void sendCommand(String command) {
		if(!portOpened) return;
		
		commandQueue.add(command);
		if(portConfirmed) sendQueuedCommand();
	}
	
	
	public void closePort() {
		if(!portOpened) return;
		
	    if (serialPort != null) {
	        // Close the port.
            serialPort.closeConnection();
            serialPort=null;
	    }

		portOpened=false;
		portConfirmed=false;
	}
	
	/**
	 * open a serial connection to a device.  We won't know it's the robot until  
	 * @param portName @error broken
	 */
	public void openPort(String portName) {
		if(portOpened) closePort();
		
		serialPort = NetworkConnectionManager.requestNewConnection(null);
		if(serialPort!=null) {
			portOpened=true;
		}
	}
	
    // Adds a listener that should be notified.
    public void addListener(SerialConnectionReadyListener listener) {
      listeners.add(listener);
    }

    // Notifies all the listeners
    private void notifyListeners() {
      for (SerialConnectionReadyListener listener : listeners) {
        listener.SerialConnectionReady(this);
      }
    }

	public JMenu getMenu() {
		JMenu subMenu = new JMenu();
	    //ButtonGroup group = new ButtonGroup();
	    
	    return subMenu;
	}
	

	public Component getGUI() {
	    // the log panel
	    log.setEditable(false);
	    log.setForeground(Color.GREEN);
	    log.setBackground(Color.BLACK);
	    logPane = new JScrollPane(log);
	    
	    return logPane;
	}

	@Override
	public void lineError(NetworkConnection arg0, int lineNumber) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void sendBufferEmpty(NetworkConnection arg0) {
		if(portConfirmed) sendQueuedCommand();
	}
}
