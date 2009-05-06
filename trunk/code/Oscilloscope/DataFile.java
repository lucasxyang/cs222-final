import java.io.*;

public class DataFile {

	String filename;
	Encoder encoder;
	double compressionRatio;
	
	public DataFile(String filename, Encoder encoder) {
		this.filename = filename;
		this.encoder = encoder;
	}
	
	public void run() {
		try {
			BufferedReader reader = new BufferedReader(new FileReader(filename));
			String line;
			while ((line = reader.readLine()) != null) {
				Integer element = Integer.parseInt(line);
				encoder.addElement(element);
				//System.out.println(encoder.getLastCompressionRatio());
			}
			compressionRatio = encoder.getTotalCompressionRatio();
		} catch (Exception e) {
			System.out.println(e.toString());
			e.printStackTrace();
		}
	}
	
	public double getCompressionRatio() {
		return compressionRatio;
	}
}
