package com.marginallyclever.communications.tcp;

import com.marginallyclever.communications.NetworkConnection;
import com.marginallyclever.communications.TransportLayer;
import com.marginallyclever.communications.TransportLayerPanel;

/**
 * Lists available TCP connections and opens a connection of that type to a robot
 *
 * @author Dan
 * @since v7.1.0.0
 */
public class TCPTransportLayer implements TransportLayer {
	public TCPTransportLayer() {}

	/**
	 * @return <code>serialConnection</code> if connection successful.  <code>null</code> on failure.
	 */
	public NetworkConnection openConnection(String connectionName) {
		/*
		// check it
		Log.message("Validating "+connectionName);
		InetAddressValidator validator = new InetAddressValidator();
		if(!validator.isValid(connectionName)) {
			Log.error("Not a valid IP Address.");
			return null;
		}
		*/
		System.out.println("Connecting to "+connectionName);
		//if(connectionName.equals(recentPort)) return null;
		TCPConnection connection = new TCPConnection(this);

		try {
			connection.openConnection(connectionName);
			System.out.println("Connect OK");
		} catch (Exception e) {
			System.out.println("Connect FAILED");
			e.printStackTrace();
			return null;
		}

		return connection;
	}

	@Override
	public TransportLayerPanel getTransportLayerPanel() {
		return new TCPTransportLayerPanel(this);
	}
}
