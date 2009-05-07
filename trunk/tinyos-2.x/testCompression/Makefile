COMPONENT = TestCompressionC
BUILD_EXTRA_DEPS = SampleMsg.py

# Dependencies
CFLAGS += -I$(TOSDIR)/lib/printf 

PFLAGS += -DSAMPLING_DMA
PFLAGS += -DTOSH_DATA_LENGTH=32

SampleMsg.py: sample.h
	mig python -target=$(PLATFORM) $(CFLAGS) -python-classname=SampleMsg sample.h sample_msg -o $@

include $(MAKERULES)

