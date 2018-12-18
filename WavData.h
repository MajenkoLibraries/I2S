#ifndef _WAV_DATA_H
#define _WAV_DATA_H

#include <I2S.h>
#include <WavData.h>

#include "WavCommon.h"

class WavData : public AudioSource {
    private:
        const uint8_t *_data;
        const uint8_t *_pos;
        const uint8_t *_end;

        wav_header *_header;

        uint32_t getNextSampleBlock8(int16_t *buf, uint32_t samps);
        uint32_t getNextSampleBlock16(int16_t *buf, uint32_t samps);

        void initData();

    public:

        WavData(const uint8_t *d) : _data(d) {}

        void initStream();
        uint32_t seekToFrame(uint32_t frame);
        uint32_t getNextSampleBlock(int16_t *buf, uint32_t nsamps);
        uint32_t getChannels();
        uint32_t getFrameLength();
};

#endif
