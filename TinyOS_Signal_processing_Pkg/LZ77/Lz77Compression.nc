interface Lz77Compression
{

command error_t Lz77Encode(int*);
event error_t Lz77EncodeDone(error_t success);
}
