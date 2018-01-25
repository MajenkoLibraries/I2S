#ifndef _I2S_H
#define _I2S_H

//#define BSIZE 1024
#define BSIZE 4096

#include <Arduino.h>
#include <DSPI.h>
#include <DFATFS.h>


class DAC {
    public:
        virtual void setGain(int8_t vol) = 0;
        virtual void setVolume(uint8_t v) = 0;
        virtual void setVolume(uint8_t l, uint8_t r) = 0;
        virtual void setBass(int8_t cut) = 0;
        virtual void setMid(int8_t cut) = 0;
        virtual void setTreble(int8_t cut) = 0;

        virtual void enable3D() = 0;
        virtual void disable3D() = 0;
        virtual void setWide3D() = 0;
        virtual void setNarrow3D() = 0;
        virtual void set3DDepth(uint8_t w) = 0;
};

class Amplifier {
    public:
        virtual void setGain(int8_t vol) = 0;
        virtual void enableAmplifier() = 0;
        virtual void disableAmplifier() = 0;
};


struct sample_s {
    const int16_t *data;    // Data in memory to play
    DFILE *file;            // File to play samples from
    uint32_t len;           // Length in samples
    float pos;              // Playback position in samples. Needs to be float to make the speed calculations easier
    float speed;            // Fractional playback speed (1.0 = normal speed)
    float vol;              // Fractional playback volume (1.0 = normal volume)
    uint8_t flags;          // Various settings for the sample
    uint32_t offset;        // Offset to start playing from
    uint32_t loop_start;    // When looping go back to here
    uint32_t loop_end;      // Loop back when you get here, and play from here for aftertouch
};

#define MAX_SAMPLES 50

// This sample is active
#define SMP_ACTIVE      0x01

// This is a stereo sample data set
#define SMP_STEREO      0x02

// Play the sample out of the left speaker
#define SMP_LEFT        0x04

// Play the sample out of the right speaker
#define SMP_RIGHT       0x08

// Loop the sample until stopper
#define SMP_LOOP        0x10

// After playing a loop play the aftertouch portion
#define SMP_AFTERTOUCH  0x20

// Sample is playing
#define SMP_PLAYING     0x80

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

        void uninitClock();
        void uninitSPI();
        void uninitDMA();  

        static int16_t mix(int16_t a, int16_t b);

        static bool doFillBuffer(int32_t *buf);

        static void __USER_ISR DMA1ISR(void);
        static void __USER_ISR DMA2ISR(void);

	public:
		I2S(uint32_t sr);
		void begin();
        void end();
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
        void setSpeed(int s, float v);
        void setVolume(int s, float v);
        void enableLoop(int s, int st, int e, bool a);
        void disableLoop(int s);
        bool isPlaying(int s);
        void setSampleRate(uint32_t r);
};

#endif
