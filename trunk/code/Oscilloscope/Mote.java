import net.tinyos.message.*;
import net.tinyos.util.*;

public class Mote implements MessageListener{

	MoteIF mote;
	Oscilloscope oscilloscope;
	int x;
	
	public Mote(Oscilloscope oscilloscope) {
		this.oscilloscope = oscilloscope;
		mote = new MoteIF(PrintStreamMessenger.err);
		mote.registerListener(new OscilloscopeMsg(), this);
		x = 0;
	}
	
	public void messageReceived(int dest_addr, Message msg) {
		if (msg instanceof OscilloscopeMsg){
			OscilloscopeMsg omsg = (OscilloscopeMsg)msg;
			for (Integer y : omsg.get_readings()) {
				oscilloscope.addData(x, y);
				x++;
				try {
					Thread.currentThread().sleep(150);
				} catch (Exception e) {
					
				}
				System.out.println(y);
			}
		}
	}
}
