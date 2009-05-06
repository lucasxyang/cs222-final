import org.jfree.ui.RefineryUtilities;


public class Main {

    public static void main(final String[] args) {
    	
        if (args.length  < 1) {
        	System.out.println("Need to specify bits per sample");
        	System.out.println("Usage:");
        	System.out.println("java -jar Oscilloscope.jar bitspersample [file]");
        	System.out.println("If you dont specify a file, it will assume you are looking for motes ");
        	System.out.println("and connect to the serial forwarder port");
        	return;
        }
        
        int bitsPerSample = Integer.parseInt(args[0]);
  
        //For motes
        if (args.length == 1) {
        	Oscilloscope demo = new Oscilloscope("Compression Ratio");
            Encoder encoder = new Encoder(bitsPerSample);
            Mote mote = new Mote(demo, encoder);
            demo.pack();
            RefineryUtilities.centerFrameOnScreen(demo);
            demo.setVisible(true);
            
        } 
        //For files
        else if (args.length == 2) {
        	Encoder encoder = new Encoder(bitsPerSample);
        	DataFile df = new DataFile(args[1], encoder);
        	df.run();
        	System.out.println(encoder.getTotalCompressionRatio());
        	
        }
    	
        /*
        for (int i = 0; i < 100; i++) {
        	try {
        		Thread.currentThread().sleep(100);
        	} catch (Exception e) {
        		
        	}
        	demo.addData(i, Math.random());
        	
        }
        */
        
    }
}
