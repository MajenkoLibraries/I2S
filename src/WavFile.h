#ifndef _WAV_FILE_H
#define _WAV_FILE_H

#include <I2S.h>
#include <DFATFS.h>

#include "WavCommon.h"

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

#endif
