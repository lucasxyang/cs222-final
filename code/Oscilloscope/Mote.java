import net.tinyos.message.*;
import net.tinyos.util.*;

public class Mote implements MessageListener{

	MoteIF mote;
	Oscilloscope oscilloscope;
	Encoder encoder;
	int x;
	
	public Mote(Oscilloscope oscilloscope, Encoder encoder) {
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
				//System.out.println(y);
			}
		}
		if (msg instanceof SampleMsg) {
			SampleMsg smsg = (SampleMsg)msg;
			for (Short element: smsg.get_buffer()) {
				encoder.addElement((int)element);
				oscilloscope.addData(x, encoder.getLastCompressionRatio());
				x++;
				System.out.println(encoder.getLastCompressionRatio());
			}
		}
	}
}
