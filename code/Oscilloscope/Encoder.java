import java.util.LinkedList;


public class Encoder {
	
	LinkedList<Integer> dataQueue;
	Block workingBlock;
	int bitsPerSampleUncompressed;
	int samplesCompressed;
	int samplesUncompressed;
	double lastCalculatedCompressionRatio;
	
	public Encoder(int bitsPerSampleUncompressed) {
		this.bitsPerSampleUncompressed = bitsPerSampleUncompressed;
		dataQueue = new LinkedList<Integer>();
		workingBlock = new Block(this.bitsPerSampleUncompressed);
		samplesCompressed = 0;
		samplesUncompressed = 0;
		lastCalculatedCompressionRatio = 0;
	}
	
	public Block addElement(Integer element) {
		if (workingBlock.doesElementFit(element)) {
			workingBlock.addElement(element);
			return null;
		} else {
			samplesCompressed += workingBlock.getCompressedSamples();
			samplesUncompressed += workingBlock.getUncompressedSamples();
			lastCalculatedCompressionRatio = workingBlock.getCompressionRatio();
			Block fullBlock = workingBlock;
			workingBlock = new Block(bitsPerSampleUncompressed);
			return fullBlock;
		}
	}
	
	public double getLastCompressionRatio() {
		return lastCalculatedCompressionRatio;
	}
	
	public double getTotalCompressionRatio() {
		return (double)samplesUncompressed/samplesCompressed;
	}
	
}
