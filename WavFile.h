#include <I2S.h>
#include <DFATFS.h>

typedef struct wav_header {
    // RIFF Header
    char riff_header[4]; // Contains "RIFF"
    uint32_t wav_size; // Size of the wav portion of the file, which follows the first 8 bytes. File size - 8
    char wave_header[4]; // Contains "WAVE"
    
    // Format Header
    char fmt_header[4]; // Contains "fmt " (includes trailing space)
    uint32_t fmt_chunk_size; // Should be 16 for PCM
    uint16_t audio_format; // Should be 1 for PCM. 3 for IEEE Float
    uint16_t num_channels;
    uint32_t sample_rate;
    uint32_t byte_rate; // Number of bytes per second. sample_rate * num_channels * Bytes Per Sample
    uint16_t sample_alignment; // num_channels * Bytes Per Sample
    uint16_t bit_depth; // Number of bits per sample
    
    // Data
    char data_header[4]; // Contains "data"
    uint32_t data_bytes; // Number of bytes in data. Number of samples * num_channels * sample byte size
    // uint8_t bytes[]; // Remainder of wave file is bytes
} wav_header;

class WavFile : public AudioSource {
    private:
        DFILE *_file;

        wav_header _header;

        bool initFile();
        uint32_t getNextSampleBlock8(int16_t *buf, uint32_t samps);
        uint32_t getNextSampleBlock16(int16_t *buf, uint32_t samps);

    public:

        WavFile(DFILE &f) : _file(&f) {}
        WavFile(DFILE *f) : _file(f) {}

        void initStream();
        uint32_t seekToFrame(uint32_t frame);
        uint32_t getNextSampleBlock(int16_t *buf, uint32_t nsamps);
        uint32_t getChannels();
        uint32_t getFrameLength();
};
