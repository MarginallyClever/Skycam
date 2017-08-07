package com.marginallyclever.Skycam;

import java.awt.BorderLayout;
import java.awt.Container;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Scanner;
import java.util.prefs.Preferences;
import java.lang.Boolean;

import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JDialog;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JMenuItem;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JSplitPane;
import javax.swing.JTextField;
import javax.swing.KeyStroke;
import javax.swing.filechooser.FileFilter;
import javax.swing.filechooser.FileNameExtensionFilter;
import javax.vecmath.Point3d;


/**
 * Skycam makes two serial connections to arduinos with adafruit stepper shields.  Each shield drives two stepper motors.
 * If a clock were drawn around the origin, The motors would be arranged at the 10:00, 2:00, 4:30, and 7:30 positions.
 * Each arduino is running a variant of the Drawbot code.   This variant is designed to work in 3D.
 * 
 * After the Java program has authenticated the robot's existence and established communications, the fist thing the java
 * program says is "Here is your location in the world and your designation."  This is done with the CONFIG command.
 * 
 * Once the robots know their part in the game, their position on the clock face can be calculated.  Each motor gets a
 * bobbin wound with string.  The four strings come together at a common point somewhere inside the cube formed by
 * the four motors and the floor.  Changing the string lengths in the right ratio will move the point to anywhere within
 * the cube limits.
 * @author danroyer
 *
 */
public class Skycam
extends JPanel
implements ActionListener, SerialConnectionReadyListener
{
	static final long serialVersionUID=1;
	
	static JFrame mainFrame;
	static Skycam singleton=null;
	
	// menus
	private JMenuBar menuBar;
	private JMenuItem buttonOpenFile, buttonExit;
    private JMenuItem [] buttonRecent = new JMenuItem[10];
	private JMenuItem buttonJogMotors, buttonMachineLimits, buttonDisconnect;
	private JMenuItem buttonStart, buttonPause, buttonHalt, buttonDrive;
	
	// serial connections
	private SkycamRobot connectionBerlin;
	private SkycamRobot connectionTokyo;
	private boolean aReady=false, bReady=false, wasConfirmed=false;

	// settings
	private Preferences prefs;
	private String[] recentFiles = {"","","","","","","","","",""};
	private boolean m1invert=false,
					m2invert=false,
					m3invert=false,
					m4invert=false;
	
	// config
	Point3d m1=new Point3d();
	Point3d m2=new Point3d();
	Point3d m3=new Point3d();
	Point3d m4=new Point3d();
	
	// files
	private boolean running=false;
	private boolean paused=true;
    private long linesTotal=0;
	private long linesProcessed=0;
	private boolean fileOpened=false;
	private ArrayList<String> gcode;
	//private float estimated_time=0;
	
	
	private Skycam() {
		prefs = Preferences.userRoot().node("Skycam");
		
		LoadConfig();
		
		connectionBerlin = new SkycamRobot("A");
		connectionTokyo = new SkycamRobot("B");
		
		connectionBerlin.addListener(this);
		connectionTokyo.addListener(this);
		
		SendConfig();
	}
	
	static public Skycam getSingleton() {
		if(singleton==null) singleton=new Skycam();
		return singleton;
	}
	
	public void SerialConnectionReady(SkycamRobot arg0) {
		if(arg0==connectionBerlin) aReady=true;
		if(arg0==connectionTokyo) bReady=true;
		
		if(aReady && bReady) {
			if(!wasConfirmed) {
				wasConfirmed=true;
				UpdateMenuBar();
			}
			aReady=bReady=false;
			SendFileCommand();
		}
	}
	
	@Override
	public void actionPerformed(ActionEvent e) {
		Object subject = e.getSource();
		
		if(subject==buttonExit) {
			quit();
			return;
		}
		if(subject==buttonOpenFile) {
			openFileDialog();
			return;
		}
		if(subject==buttonMachineLimits) {
			updateMachineLimits();
			return;
		}
		if(subject==buttonDisconnect) {
			connectionBerlin.closePort();
			connectionTokyo.closePort();
			return;
		}
		if(subject==buttonStart) {
			//if(fileOpened) OpenFile(recentFiles[0]);
			if(fileOpened) {
				paused=false;
				running=true;
				UpdateMenuBar();
				linesProcessed=0;
				//previewPane.setRunning(running);
				//previewPane.setLinesProcessed(linesProcessed);
				//statusBar.Start();
				SendFileCommand();
			}
			return;
		}
		if( subject == buttonPause ) {
			if(running) {
				if(paused==true) {
					buttonPause.setText("Pause");
					paused=false;
					// @TODO: if the robot is not ready to unpause, this might fail and the program would appear to hang.
					SendFileCommand();
				} else {
					buttonPause.setText("Unpause");
					paused=true;
				}
			}
			return;
		}
		if( subject == buttonJogMotors ) {
			JogMotors();
			return;
		}
		if( subject == buttonDrive ) {
			Drive();
			return;
		}
		if( subject == buttonHalt ) {
			Halt();
			return;
		}
		
		int i;
		for(i=0;i<10;++i) {
			if(subject == buttonRecent[i]) {
				OpenFile(recentFiles[i]);
				return;
			}
		}
	}
	
	/**
	 * stop sending commands to the robot.
	 * @todo add an e-stop command?
	 */
	public void Halt() {
		running=false;
		paused=false;
	    linesProcessed=0;
	    //previewPane.setLinesProcessed(0);
		//previewPane.setRunning(running);
		UpdateMenuBar();
	}

	private void GoHome() {
		SendLineToRobot("G00 X0 Y0 Z0");
	}
	
	/**
	 * Open the driving dialog
	 */
	public void Drive() {
		JDialog driver = new JDialog(mainFrame,"Manual Control",true);
		driver.setLayout(new GridBagLayout());
		GridBagConstraints c = new GridBagConstraints();
		
		JButton home = new JButton("HOME");
		
		JButton up1 = new JButton("Y1");
		JButton up10 = new JButton("Y10");
		JButton up100 = new JButton("Y100");
		
		JButton down1 = new JButton("Y-1");
		JButton down10 = new JButton("Y-10");
		JButton down100 = new JButton("Y-100");
		
		JButton left1 = new JButton("X-1");
		JButton left10 = new JButton("X-10");
		JButton left100 = new JButton("X-100");
		
		JButton right1 = new JButton("X1");
		JButton right10 = new JButton("X10");
		JButton right100 = new JButton("X100");
		
		JButton in1 = new JButton("Z-1");
		JButton in10 = new JButton("Z-10");
		JButton in100 = new JButton("Z-100");
		
		JButton out1 = new JButton("Z1");
		JButton out10 = new JButton("Z10");
		JButton out100 = new JButton("Z100");
		
		JButton center = new JButton("CENTERED");
		
		c.gridx=3;	c.gridy=0;	driver.add(up100,c);
		c.gridx=3;	c.gridy=1;	driver.add(up10,c);
		c.gridx=3;	c.gridy=2;	driver.add(up1,c);
		c.gridx=3;	c.gridy=4;	driver.add(down1,c);
		c.gridx=3;	c.gridy=5;	driver.add(down10,c);
		c.gridx=3;	c.gridy=6;	driver.add(down100,c);

		c.gridx=3;	c.gridy=3;	driver.add(center,c);
		
		c.gridx=0;	c.gridy=3;	driver.add(left100,c);
		c.gridx=1;	c.gridy=3;	driver.add(left10,c);
		c.gridx=2;	c.gridy=3;	driver.add(left1,c);
		c.gridx=4;	c.gridy=3;	driver.add(right1,c);
		c.gridx=5;	c.gridy=3;	driver.add(right10,c);
		c.gridx=6;	c.gridy=3;	driver.add(right100,c);
		
		c.gridx=7;	c.gridy=0;	driver.add(out100,c);
		c.gridx=7;	c.gridy=1;	driver.add(out10,c);
		c.gridx=7;	c.gridy=2;	driver.add(out1,c);
		c.gridx=7;	c.gridy=4;	driver.add(in1,c);
		c.gridx=7;	c.gridy=5;	driver.add(in10,c);
		c.gridx=7;	c.gridy=6;	driver.add(in100,c);

		c.gridx=0;  c.gridy=6;  driver.add(home,c);
		
		ActionListener driveButtons = new ActionListener() {
			  public void actionPerformed(ActionEvent e) {
					Object subject = e.getSource();
					JButton b = (JButton)subject;
					String t=b.getText();
					if(t=="HOME") {
						GoHome();
					} else
					if(t=="CENTERED") {
						SendLineToRobot("TELEPORT X0 Y0 Z0");
					} else {
						SendLineToRobot("G91");
						SendLineToRobot("G00 "+b.getText());
						SendLineToRobot("G90");
					}
			  }
			};
		
		   up1.addActionListener(driveButtons);		   up10.addActionListener(driveButtons);	   up100.addActionListener(driveButtons);
		 down1.addActionListener(driveButtons);		 down10.addActionListener(driveButtons);	 down100.addActionListener(driveButtons);
		 left1.addActionListener(driveButtons);		 left10.addActionListener(driveButtons);	 left100.addActionListener(driveButtons);
		right1.addActionListener(driveButtons);		right10.addActionListener(driveButtons);	right100.addActionListener(driveButtons);
		   in1.addActionListener(driveButtons);		   in10.addActionListener(driveButtons);	   in100.addActionListener(driveButtons);
		  out1.addActionListener(driveButtons);		  out10.addActionListener(driveButtons);	  out100.addActionListener(driveButtons);
		center.addActionListener(driveButtons);
		home.addActionListener(driveButtons);
		driver.pack();
		driver.setVisible(true);
	}
	
	
	protected void JogMotors() {
		JDialog driver = new JDialog(mainFrame,"Jog Motors",true);
		driver.setLayout(new GridBagLayout());
		GridBagConstraints c = new GridBagConstraints();
		
		final JButton buttonAneg = new JButton("IN");
		final JButton buttonApos = new JButton("OUT");
		final JCheckBox m1i = new JCheckBox("Invert",m1invert);
		
		final JButton buttonBneg = new JButton("IN");
		final JButton buttonBpos = new JButton("OUT");
		final JCheckBox m2i = new JCheckBox("Invert",m2invert);
		
		final JButton buttonCneg = new JButton("IN");
		final JButton buttonCpos = new JButton("OUT");
		final JCheckBox m3i = new JCheckBox("Invert",m3invert);
		
		final JButton buttonDneg = new JButton("IN");
		final JButton buttonDpos = new JButton("OUT");
		final JCheckBox m4i = new JCheckBox("Invert",m4invert);

		c.gridx=0;	c.gridy=0;	driver.add(new JLabel("A"),c);
		c.gridx=0;	c.gridy=1;	driver.add(new JLabel("B"),c);
		c.gridx=0;	c.gridy=2;	driver.add(new JLabel("C"),c);
		c.gridx=0;	c.gridy=3;	driver.add(new JLabel("D"),c);
		
		c.gridx=1;	c.gridy=0;	driver.add(buttonAneg,c);
		c.gridx=1;	c.gridy=1;	driver.add(buttonBneg,c);
		c.gridx=1;	c.gridy=2;	driver.add(buttonCneg,c);
		c.gridx=1;	c.gridy=3;	driver.add(buttonDneg,c);
		
		c.gridx=2;	c.gridy=0;	driver.add(buttonApos,c);
		c.gridx=2;	c.gridy=1;	driver.add(buttonBpos,c);
		c.gridx=2;	c.gridy=2;	driver.add(buttonCpos,c);
		c.gridx=2;	c.gridy=3;	driver.add(buttonDpos,c);

		c.gridx=3;	c.gridy=0;	driver.add(m1i,c);
		c.gridx=3;	c.gridy=1;	driver.add(m2i,c);
		c.gridx=3;	c.gridy=2;	driver.add(m3i,c);
		c.gridx=3;	c.gridy=3;	driver.add(m4i,c);
		
		ActionListener driveButtons = new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				Object subject = e.getSource();
				if(subject == buttonApos) SendLineToRobot("D00 A100");
				if(subject == buttonAneg) SendLineToRobot("D00 A-100");
				
				if(subject == buttonBpos) SendLineToRobot("D00 B100");
				if(subject == buttonBneg) SendLineToRobot("D00 B-100");
				
				if(subject == buttonCpos) SendLineToRobot("D00 C100");
				if(subject == buttonCneg) SendLineToRobot("D00 C-100");
				
				if(subject == buttonDpos) SendLineToRobot("D00 D100");
				if(subject == buttonDneg) SendLineToRobot("D00 D-100");
			}
		};

		ActionListener invertButtons = new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				m1invert = m1i.isSelected();
				m2invert = m2i.isSelected();
				m3invert = m3i.isSelected();
				m4invert = m4i.isSelected();
				
				SaveConfig();
				SendConfig();
			}
		};
		
		buttonApos.addActionListener(driveButtons);
		buttonAneg.addActionListener(driveButtons);
		
		buttonBpos.addActionListener(driveButtons);
		buttonBneg.addActionListener(driveButtons);
		
		buttonCpos.addActionListener(driveButtons);
		buttonCneg.addActionListener(driveButtons);

		buttonDpos.addActionListener(driveButtons);
		buttonDneg.addActionListener(driveButtons);

		m1i.addActionListener(invertButtons);
		m2i.addActionListener(invertButtons);
		m3i.addActionListener(invertButtons);
		m4i.addActionListener(invertButtons);
		
		driver.pack();
		driver.setVisible(true);
	}
	
	// Take the next line from the file and send it to the robot, if permitted. 
	public void SendFileCommand() {
		if(running==false || paused==true || fileOpened==false || IsConfirmed()==false || linesProcessed>=linesTotal) return;
		
		String line;
		do {			
			// are there any more commands?
			line=gcode.get((int)linesProcessed++).trim();
			//previewPane.setLinesProcessed(linesProcessed);
			//statusBar.SetProgress(linesProcessed, linesTotal);
			// loop until we find a line that gets sent to the robot, at which point we'll
			// pause for the robot to respond.  Also stop at end of file.
		} while(!SendLineToRobot(line) && linesProcessed<linesTotal);
		
		if(linesProcessed==linesTotal) {
			// end of file
			Halt();
		}
	}
	
	/**
	 * Processes a single instruction meant for the robot.
	 * @param line
	 * @return true if the command is sent to the robot.
	 */
	public boolean SendLineToRobot(String line) {
		// tool change request?
		String [] tokens = line.split("\\s");

		// tool change?
		if(Arrays.asList(tokens).contains("M06") || Arrays.asList(tokens).contains("M6")) {
			for(int i=0;i<tokens.length;++i) {
				if(tokens[i].startsWith("T")) {
					JOptionPane.showMessageDialog(null,"Please change to tool #"+tokens[i].substring(1)+" and click OK.");
				}
			}
			// still ready to send
			return false;
		}
		
		// end of program?
		if(tokens[0]=="M02" || tokens[0]=="M2") {
			Halt();
			return false;
		}
		
		// other machine code to ignore?
		if(tokens[0].startsWith("M")) {
			//Log(line+NL);
			return false;
		} 

		// contains a comment?  if so remove it
		int index=line.indexOf('(');
		if(index!=-1) {
			//String comment=line.substring(index+1,line.lastIndexOf(')'));
			//Log("* "+comment+NL);
			line=line.substring(0,index).trim();
			if(line.length()==0) {
				// entire line was a comment.
				return false;  // still ready to send
			}
		}

		// send relevant part of line to the robot
		connectionBerlin.sendCommand(line);
		connectionTokyo.sendCommand(line);
		
		return true;
	}

	protected void LoadConfig() {
		m1.x=Double.valueOf(prefs.get("m1x", "0"));
		m1.y=Double.valueOf(prefs.get("m1y", "0"));
		m1.z=Double.valueOf(prefs.get("m1z", "0"));
		m1invert=Boolean.parseBoolean(prefs.get("m1invert", "false"));

		m2.x=Double.valueOf(prefs.get("m2x", "0"));
		m2.y=Double.valueOf(prefs.get("m2y", "0"));
		m2.z=Double.valueOf(prefs.get("m2z", "0"));
		m2invert=Boolean.parseBoolean(prefs.get("m2invert", "false"));

		m3.x=Double.valueOf(prefs.get("m3x", "0"));
		m3.y=Double.valueOf(prefs.get("m3y", "0"));
		m3.z=Double.valueOf(prefs.get("m3z", "0"));
		m3invert=Boolean.parseBoolean(prefs.get("m3invert", "false"));

		m4.x=Double.valueOf(prefs.get("m4x", "0"));
		m4.y=Double.valueOf(prefs.get("m4y", "0"));
		m4.z=Double.valueOf(prefs.get("m4z", "0"));
		m4invert=Boolean.parseBoolean(prefs.get("m4invert", "false"));
		
		GetRecentFiles();
	}

	protected void SaveConfig() {
		prefs.put("m1x",String.valueOf(m1.x));
		prefs.put("m1y",String.valueOf(m1.y));
		prefs.put("m1z",String.valueOf(m1.z));
		prefs.put("m1invert",Boolean.toString(m1invert));

		prefs.put("m2x",String.valueOf(m2.x));
		prefs.put("m2y",String.valueOf(m2.y));
		prefs.put("m2z",String.valueOf(m2.z));
		prefs.put("m2invert",Boolean.toString(m2invert));

		prefs.put("m3x",String.valueOf(m3.x));
		prefs.put("m3y",String.valueOf(m3.y));
		prefs.put("m3z",String.valueOf(m3.z));
		prefs.put("m3invert",Boolean.toString(m3invert));

		prefs.put("m4x",String.valueOf(m4.x));
		prefs.put("m4y",String.valueOf(m4.y));
		prefs.put("m4z",String.valueOf(m4.z));
		prefs.put("m4invert",Boolean.toString(m4invert));
		
		GetRecentFiles();
	}

	private void SendConfig() {
		connectionBerlin.sendCommand("CONFIG A"+String.valueOf(m1.x)  // position of 10:00 motor
				                     +" B"+String.valueOf(m1.y)
				                     +" C"+String.valueOf(m1.z)
				                     +" D"+String.valueOf(m2.x)  // position of 2:00 motor
				                     +" E"+String.valueOf(m2.y)
				                     +" F"+String.valueOf(m2.z)
				                     +" GA HB"
				                     +" I"+(m1invert?"-1":"1")
				                     +" J"+(m2invert?"-1":"1"));  // name of motors, inversion
		 
		connectionTokyo.sendCommand("CONFIG A"+String.valueOf(m3.x)  // position of 4:30 motor
					                 +" B"+String.valueOf(m3.y)
					                 +" C"+String.valueOf(m3.z)
					                 +" D"+String.valueOf(m4.x)  // position of 7:30 motor
					                 +" E"+String.valueOf(m4.y)
					                 +" F"+String.valueOf(m4.z)
					                 +" GC HD"
				                     +" I"+(m3invert?"-1":"1")
				                     +" J"+(m4invert?"-1":"1"));  // name of motors, inversion
		SendLineToRobot("TELEPORT X0 Y0 Z0");	
	}
	
	protected void updateMachineLimits() {
		JDialog driver = new JDialog(mainFrame,"Motor locations",true);
		driver.setLayout(new GridLayout(6,4));
		
		JTextField m1x = new JTextField(String.valueOf(m1.x));
		JTextField m1y = new JTextField(String.valueOf(m1.y));
		JTextField m1z = new JTextField(String.valueOf(m1.z));
		
		JTextField m2x = new JTextField(String.valueOf(m2.x));
		JTextField m2y = new JTextField(String.valueOf(m2.y));
		JTextField m2z = new JTextField(String.valueOf(m2.z));
		
		JTextField m3x = new JTextField(String.valueOf(m3.x));
		JTextField m3y = new JTextField(String.valueOf(m3.y));
		JTextField m3z = new JTextField(String.valueOf(m3.z));
		
		JTextField m4x = new JTextField(String.valueOf(m4.x));
		JTextField m4y = new JTextField(String.valueOf(m4.y));
		JTextField m4z = new JTextField(String.valueOf(m4.z));

		JButton ok = new JButton("Ok");
		JButton cancel = new JButton("Cancel");

		driver.add(new JLabel(" "));
		driver.add(new JLabel("X"));
		driver.add(new JLabel("Y"));
		driver.add(new JLabel("Z"));
		driver.add(new JLabel("A"));
		driver.add(m1x);
		driver.add(m1y);
		driver.add(m1z);
		driver.add(new JLabel("B"));
		driver.add(m2x);
		driver.add(m2y);
		driver.add(m2z);
		driver.add(new JLabel("C"));
		driver.add(m3x);
		driver.add(m3y);
		driver.add(m3z);
		driver.add(new JLabel("D"));
		driver.add(m4x);
		driver.add(m4y);
		driver.add(m4z);

		driver.add(new JLabel(" "));
		driver.add(new JLabel(" "));
		driver.add(ok);
		driver.add(cancel);

		ok.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				m1.x = Double.valueOf(m1x.getText());
				m1.y = Double.valueOf(m1y.getText());
				m1.z = Double.valueOf(m1z.getText());
				
				m2.x = Double.valueOf(m2x.getText());
				m2.y = Double.valueOf(m2y.getText());
				m2.z = Double.valueOf(m2z.getText());
				m3.x = Double.valueOf(m3x.getText());
				m3.y = Double.valueOf(m3y.getText());
				m3.z = Double.valueOf(m3z.getText());
				
				m4.x = Double.valueOf(m4x.getText());
				m4.y = Double.valueOf(m4y.getText());
				m4.z = Double.valueOf(m4z.getText());
				
				SaveConfig();
				SendConfig();
				driver.dispose();
			}
		});
		
		cancel.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				driver.dispose();
			}
		});

		driver.pack();
		driver.setVisible(true);
	}

	private void CloseFile() {
		if(fileOpened==true) {
			fileOpened=false;
		}
	}
	
	/**
	 * Opens a file.  If the file can be opened, get a drawing time estimate, update recent files list, and repaint the preview tab.
	 * @param filename what file to open
	 */
	public void OpenFile(String filename) {
		CloseFile();

	    try {
	    	Scanner scanner = new Scanner(new FileInputStream(filename));
	    	linesTotal=0;
	    	gcode = new ArrayList<String>();
		    try {
		      while (scanner.hasNextLine()) {
		    	  gcode.add(scanner.nextLine());
		    	  ++linesTotal;
		      }
		    }
		    finally{
		      scanner.close();
		    }
	    }
	    catch(IOException e) {
	    	RemoveRecentFile(filename);
	    	return;
	    }
	    
	    //previewPane.setGCode(gcode);
	    fileOpened=true;
	   	UpdateRecentFiles(filename);

	   	//EstimateDrawTime();
	    Halt();
	}
	
	/**
	 * changes the order of the recent files list in the File submenu, saves the updated prefs, and refreshes the menus.
	 * @param filename the file to push to the top of the list.
	 */
	public void UpdateRecentFiles(String filename) {
		int cnt = recentFiles.length;
		String [] newFiles = new String[cnt];
		
		newFiles[0]=filename;
		
		int i,j=1;
		for(i=0;i<cnt;++i) {
			if(!filename.equals(recentFiles[i]) && recentFiles[i] != "") {
				newFiles[j++] = recentFiles[i];
				if(j == cnt ) break;
			}
		}

		recentFiles=newFiles;

		// update prefs
		for(i=0;i<cnt;++i) {
			if(!recentFiles[i].isEmpty()) {
				prefs.put("recent-files-"+i, recentFiles[i]);
			}
		}
		
		UpdateMenuBar();
	}
	
	// A file failed to load.  Remove it from recent files, refresh the menu bar.
	public void RemoveRecentFile(String filename) {
		int i;
		for(i=0;i<recentFiles.length-1;++i) {
			if(recentFiles[i]==filename) {
				break;
			}
		}
		for(;i<recentFiles.length-1;++i) {
			recentFiles[i]=recentFiles[i+1];
		}
		recentFiles[recentFiles.length-1]="";

		// update prefs
		for(i=0;i<recentFiles.length;++i) {
			if(!recentFiles[i].isEmpty()) {
				prefs.put("recent-files-"+i, recentFiles[i]);
			}
		}
		
		UpdateMenuBar();
	}
	
	// Load recent files from prefs
	public void GetRecentFiles() {
		int i;
		for(i=0;i<recentFiles.length;++i) {
			recentFiles[i] = prefs.get("recent-files-"+i, recentFiles[i]);
		}
	}
	
	// creates a file open dialog. If you don't cancel it opens that file.
	public void openFileDialog() {
	    // Note: source for ExampleFileFilter can be found in FileChooserDemo,
	    // under the demo/jfc directory in the Java 2 SDK, Standard Edition.
		String filename = (recentFiles[0].length()>0) ? filename=recentFiles[0] : "";

		FileFilter filterImage  = new FileNameExtensionFilter("Images (jpg/bmp/png/gif)", "jpg", "jpeg", "png", "wbmp", "bmp", "gif");
		FileFilter filterGCODE = new FileNameExtensionFilter("GCODE files (ngc)", "ngc");
		 
		JFileChooser fc = new JFileChooser(new File(filename));
		fc.addChoosableFileFilter(filterImage);
		fc.addChoosableFileFilter(filterGCODE);
	    if(fc.showOpenDialog(this) == JFileChooser.APPROVE_OPTION) {
	    	OpenFile(fc.getSelectedFile().getAbsolutePath());
	    }
	}
	
	private boolean IsConfirmed() {
		return connectionBerlin.portConfirmed && connectionTokyo.portConfirmed;
	}
	
	private void UpdateMenuBar() {
		JMenu menu;
        JMenu subMenu;
        
        menuBar.removeAll();
		
        // file menu.
        menu = new JMenu("File");
        menu.setMnemonic(KeyEvent.VK_F);
        menuBar.add(menu);
 
        buttonOpenFile = new JMenuItem("Open File...",KeyEvent.VK_O);
        buttonOpenFile.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_O, ActionEvent.ALT_MASK));
        buttonOpenFile.getAccessibleContext().setAccessibleDescription("Open a g-code file...");
        buttonOpenFile.addActionListener(this);
        menu.add(buttonOpenFile);

        menu.addSeparator();

        // list recent files
        GetRecentFiles();
        if(recentFiles.length>0) {
        	// list files here
        	int i;
        	for(i=0;i<recentFiles.length;++i) {
        		if(recentFiles[i].length()==0) break;
            	buttonRecent[i] = new JMenuItem((1+i) + " "+recentFiles[i],KeyEvent.VK_1+i);
            	if(buttonRecent[i]!=null) {
            		buttonRecent[i].addActionListener(this);
            		menu.add(buttonRecent[i]);
            	}
        	}
        	if(i!=0) menu.addSeparator();
        }

        buttonExit = new JMenuItem("Exit",KeyEvent.VK_Q);
        buttonExit.getAccessibleContext().setAccessibleDescription("Goodbye...");
        buttonExit.addActionListener(this);
        menu.add(buttonExit);

        menuBar.add(menu);
        
        // settings menu
        menu = new JMenu("Settings");
        menu.setMnemonic(KeyEvent.VK_T);
        menu.getAccessibleContext().setAccessibleDescription("Adjust the robot settings.");

        subMenu = connectionBerlin.getMenu();
        subMenu.setText("Arduino 1 Port");
        menu.add(subMenu);
        
        subMenu = connectionTokyo.getMenu();
        subMenu.setText("Arduino 2 Port");
        menu.add(subMenu);

        buttonMachineLimits = new JMenuItem("Set Machine Limits",KeyEvent.VK_M);
        buttonMachineLimits.addActionListener(this);
        menu.add(buttonMachineLimits);

        buttonJogMotors = new JMenuItem("Jog Motors",KeyEvent.VK_J);
        buttonJogMotors.addActionListener(this);
        menu.add(buttonJogMotors);

        menu.addSeparator();
        
        buttonDisconnect = new JMenuItem("Disconnect from arduinos",KeyEvent.VK_A);
        buttonDisconnect.addActionListener(this);
        menu.add(buttonDisconnect);
        
        menuBar.add(menu);

        // action menu
        menu = new JMenu("Action");
        menu.setMnemonic(KeyEvent.VK_A);
        menu.getAccessibleContext().setAccessibleDescription("Control robot actions.");
        menu.setEnabled(IsConfirmed());

        buttonStart = new JMenuItem("Start",KeyEvent.VK_S);
        buttonStart.getAccessibleContext().setAccessibleDescription("Start sending g-code");
        buttonStart.addActionListener(this);
    	buttonStart.setEnabled(IsConfirmed() && !running);
        menu.add(buttonStart);

        buttonPause = new JMenuItem("Pause",KeyEvent.VK_P);
        buttonPause.getAccessibleContext().setAccessibleDescription("Pause sending g-code");
        buttonPause.addActionListener(this);
        buttonPause.setEnabled(IsConfirmed() && running);
        menu.add(buttonPause);

        buttonHalt = new JMenuItem("Halt",KeyEvent.VK_H);
        buttonHalt.getAccessibleContext().setAccessibleDescription("Halt sending g-code");
        buttonHalt.addActionListener(this);
        buttonHalt.setEnabled(IsConfirmed() && running);
        menu.add(buttonHalt);

        menu.addSeparator();

        buttonDrive = new JMenuItem("Drive Manually",KeyEvent.VK_R);
        buttonDrive.getAccessibleContext().setAccessibleDescription("Etch-a-sketch style driving");
        buttonDrive.addActionListener(this);
        buttonDrive.setEnabled(IsConfirmed() && !running);
        menu.add(buttonDrive);

        menuBar.add(menu);
        
        // finish
        menuBar.updateUI();
	}
	
	public JMenuBar CreateMenuBar() {
        // If the menu bar exists, empty it.  If it doesn't exist, create it.
        menuBar = new JMenuBar();

        UpdateMenuBar();
        
        return menuBar;
	}
	
	private Container CreateContentPane() {
        JPanel contentPane = new JPanel(new BorderLayout());
        contentPane.setOpaque(true);

        JSplitPane split = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT);
        split.add(connectionBerlin.getGUI());
        split.add(connectionTokyo.getGUI());
        split.setDividerSize(8);
		split.setResizeWeight(0.5);
		split.setDividerLocation(0.5);
        
        contentPane.add(split,BorderLayout.CENTER);
        
        return contentPane;
	}
    
    // Create the GUI and show it.  For thread safety, this method should be invoked from the event-dispatching thread.
    private static void CreateAndShowGUI() {
        //Create and set up the window.
    	mainFrame = new JFrame("SkyCam");
        mainFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
 
        //Create and set up the content pane.
        Skycam demo = Skycam.getSingleton();
        mainFrame.setJMenuBar(demo.CreateMenuBar());
        mainFrame.setContentPane(demo.CreateContentPane());
 
        //Display the window.
        mainFrame.setSize(1100,500);
        mainFrame.setVisible(true);
    }
    
    protected void quit() {
		mainFrame.dispose();
    }
    
    public static void main(String[] args) {
	    //Schedule a job for the event-dispatching thread:
	    //creating and showing this application's GUI.
	    javax.swing.SwingUtilities.invokeLater(new Runnable() {
	        public void run() {
	            CreateAndShowGUI();
	        }
	    });
    }
}
