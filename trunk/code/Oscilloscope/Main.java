import org.jfree.ui.RefineryUtilities;


public class Main {

    public static void main(final String[] args) {
    	
        final Oscilloscope demo = new Oscilloscope("Compression Ratio");
        Mote mote = new Mote(demo);
        demo.pack();
        RefineryUtilities.centerFrameOnScreen(demo);
        demo.setVisible(true);

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
