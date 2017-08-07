package com.marginallyclever.communications;

import java.awt.Component;
import java.awt.GridLayout;

import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JTabbedPane;

import com.marginallyclever.communications.serial.SerialTransportLayer;
import com.marginallyclever.communications.tcp.TCPTransportLayer;
//import com.marginallyclever.communications.tcp.TCPTransportLayer;

/**
 * Handles requests between the UI and the various transport layers 
 * @author Dan Royer
 *
 */
public class NetworkConnectionManager {
	static private SerialTransportLayer serial = new SerialTransportLayer();
	static private TCPTransportLayer tcp = new TCPTransportLayer();
	
	/**
	 * create a GUI to give the user transport layer options.
	 * @param parent the root gui component
	 * @return a new connection or null.
	 */
	static public NetworkConnection requestNewConnection(Component parent) {
		JPanel top = new JPanel();
		top.setLayout(new GridLayout(0,1));
		JTabbedPane tabs = new JTabbedPane();
		top.add(tabs);
		// TODO translate me?
		tabs.addTab("USB", serial.getTransportLayerPanel());
		// TODO translate me?
		tabs.addTab("TCP/IP", tcp.getTransportLayerPanel());

		int result = JOptionPane.showConfirmDialog(parent, top, "Connect", JOptionPane.OK_CANCEL_OPTION, JOptionPane.PLAIN_MESSAGE);
		if (result == JOptionPane.OK_OPTION) {
			Component c = tabs.getSelectedComponent();
			if(c instanceof TransportLayerPanel) {
				return ((TransportLayerPanel)c).openConnection();
			}
		}
		// cancelled connect
		return null;
	}
}
