import net.tinyos.message.*;
import net.tinyos.util.*;

public class Mote implements MessageListener{

	MoteIF mote;
	Oscilloscope oscilloscope;
	Encoder encoder;
	int x;
	
	public Mote(Oscilloscope oscilloscope, Encoder encoder) {
		this.oscilloscope = oscilloscope;
		this.encoder = encoder;
		mote = new MoteIF(PrintStreamMessenger.err);
		//mote.registerListener(new OscilloscopeMsg(), this);
		mote.registerListener(new SampleMsg(), this);
		x = 0;
	}
	
	public void messageReceived(int dest_addr, Message msg) {
		//System.out.println("Message received");
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
			short[] buffer = smsg.get_buffer();
			for (int i = 0; i < buffer.length; i+=2) {
				int element = buffer[i] + (buffer[i+1] << 8);
				//System.out.println(element);
				encoder.addElement((int)element);
				if (encoder.resultedInNewBlock()){
					oscilloscope.addData(x, encoder.getLastCompressionRatio());
					x++;
					//System.out.println(encoder.getLastCompressionRatio());
					
				}
			}
		}
	}
}
