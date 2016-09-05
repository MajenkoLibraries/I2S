#ifndef _I2S_H
#define _I2S_H

#define BSIZE 1024

#include <Arduino.h>
#include <DSPI.h>
#include <DFATFS.h>

struct sample_s {
    const int16_t *data;    // Data in memory to play
    DFILE *file;            // File to play samples from
    uint32_t len;           // Length in samples
    float pos;              // Playback position in samples. Needs to be float to make the speed calculations easier
    float speed;            // Fractional playback speed (1.0 = normal speed)
    float vol;              // Fractional playback volume (1.0 = normal volume)
    uint8_t flags;          // Various settings for the sample
    uint32_t offset;        // Offset to start playing or looping from
};

#define MAX_SAMPLES 10

// This sample is active
#define SMP_ACTIVE  0x01

// This is a stereo sample data set
#define SMP_STEREO  0x02

// Play the sample out of the left speaker
#define SMP_LEFT    0x04

// Play the sample out of the right speaker
#define SMP_RIGHT   0x08

// Loop the sample until stopper
#define SMP_LOOP    0x10

// Sample is playing
#define SMP_PLAYING 0x20

typedef struct sample_s sample;

class I2S {
	private:
        static uint32_t _sampleRate;
        static uint32_t _bufferPos;

        static sample _samples[MAX_SAMPLES];

        // DMA buffers must be "coherent" to avoid caching in more advanced chips
        static int32_t __attribute__((coherent)) _bufferA[BSIZE];
        static int32_t __attribute__((coherent)) _bufferB[BSIZE];
        static bool _bufferAFull;
        static bool _bufferBFull;

        void initClock();
        void initSPI();
        void initDMA();  

        static bool doFillBuffer(int32_t *buf);

        static void __USER_ISR DMA1ISR(void);
        static void __USER_ISR DMA2ISR(void);

	public:
		I2S(uint32_t sr);
		void begin();
        bool ready() { return ((!_bufferAFull) || (!_bufferBFull)); }
        bool fill(int16_t sample);
        bool fill(int16_t s1, int16_t s2);
        bool fillMono(int16_t sample);

        int playStereo(const int16_t *samples, uint32_t len, float volume, float speed, uint32_t offset = 0);
        int playStereo(DFILE &file, float volume, float speed, uint32_t offset = 0);
        int playMono(const int16_t *samples, uint32_t len, float volume, float speed, uint32_t offset = 0);
        int playMono(DFILE &file, float volume, float speed, uint32_t offset = 0);
        int playMonoLeft(const int16_t *samples, uint32_t len, float volume, float speed, uint32_t offset = 0);
        int playMonoLeft(DFILE &file, float volume, float speed, uint32_t offset = 0);
        int playMonoRight(const int16_t *samples, uint32_t len, float volume, float speed, uint32_t offset = 0);
        int playMonoRight(DFILE &file, float volume, float speed, uint32_t offset = 0);
        void stop(int s);
        void resume(int s);
        void cancel(int s);
        void setVolume(int s, float v);
        void enableLoop(int s);
        void disableLoop(int s);
};

#endif
