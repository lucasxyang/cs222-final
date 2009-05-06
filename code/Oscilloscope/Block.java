import java.util.ArrayList;

public class Block {
	
	//This is the max bytes available to encode
	//the deltas
	static final int MAX_PAYLOAD_BYTES = SampleMsg.DEFAULT_MESSAGE_SIZE;
	static final int OVERHEAD_BYTES = 3;
	static final int MAX_DATA_SIZE_BYTES = MAX_PAYLOAD_BYTES - OVERHEAD_BYTES;
	
	int bitsPerSampleUncompressed;
	int blockSize;
	int bitsPerElement;
	int firstElement;
	int lastElement;
	ArrayList<Integer> deltas;
	
	
	public Block(int bitsPerSampleUncompressed) {
		this.bitsPerSampleUncompressed = bitsPerSampleUncompressed;
		this.blockSize = 0;
		this.bitsPerElement = 0;
		this.firstElement = 0;
		this.deltas = new ArrayList<Integer>();
	}
	
	public void addElement(Integer element) {
		if (blockSize == 0) {
			firstElement = element;
			lastElement = element;
			blockSize++;
		} else {
			int delta = lastElement - element;
			deltas.add(delta);
			lastElement = element;
			blockSize++;
			int bitsNeededForDelta = calculateBitsForDelta(delta);
			if (bitsNeededForDelta > bitsPerElement) {
				bitsPerElement = bitsNeededForDelta;
			}
		}
	}
	
	public boolean doesElementFit(Integer element) {
		int delta = lastElement - element;
		int bitsNeededForDelta = calculateBitsForDelta(delta);
	
		int deltaCount = blockSize - 1 + 1;
		int newDeltaBits = deltaCount * bitsNeededForDelta;
		if (newDeltaBits > (MAX_DATA_SIZE_BYTES * 8)) {
			return false;
		} else {
			return true;
		}
	
	}
	
	public double getCompressionRatio() {
		return (double)getUncompressedSamples()/getCompressedSamples();
	}
	
	public int getOverheadBytes() {
		return OVERHEAD_BYTES;
	}
	
	public int getUncompressedSamples() {
		return MAX_PAYLOAD_BYTES * 8 / bitsPerSampleUncompressed;
	}
	
	public int getCompressedSamples() {
		return blockSize;
	}
	
	public int calculateBitsForDelta(Integer delta) {
		if (delta == 0)
			return 1;
		else
			return (int)Math.ceil(Math.log(Math.abs(delta))/Math.log(2));
	}

	public int getBitsPerElement() {
		return bitsPerElement;
	}

	public void setBitsPerElement(int bitsPerElement) {
		this.bitsPerElement = bitsPerElement;
	}

	public int getBlockSize() {
		return blockSize;
	}

	public void setBlockSize(int blockSize) {
		this.blockSize = blockSize;
	}
	
	
	
	
}
