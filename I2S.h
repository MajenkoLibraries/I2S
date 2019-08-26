#ifndef _I2S_H
#define _I2S_H

//#define BSIZE 1024
#define BSIZE 16384

#include <Arduino.h>
#include <DSPI.h>
#include <DFATFS.h>

class AudioSource {
    public:
        /*! Get the next N frames as signed 16-bit samples. The audio source must convert the samples
         *  internally into signed 16 bit values and fill the array as it sees fit. The I2C system
         *  will interpret the sample sequence into frames using getChannels() to demultiplex the
         *  data into left and right or duplicate into both.
         *
         *  Returns the number of samples actually placed into the samples array.
         */
        virtual uint32_t getNextSampleBlock(int16_t *samples, uint32_t maxlen) = 0;

        /*! Returns the number of frames within the entire audio source.  A frame consists of one sample
         *  per channel.
         */
        virtual uint32_t getFrameLength() = 0;

        /*! Return the number of channels in a single frame. Either 1 or 2 are valid values currently.
         */
        virtual uint32_t getChannels() = 0;

        /*! Position the start of the next "getNextSampleBlock" call to the specified frame. Returns
         *  the actual frame number seeked to (sooked to? enseeken to?) in the case the source cannot
         *  get to the exact frame specified.
         */
        virtual uint32_t seekToFrame(uint32_t frame) = 0;

        /*! Initialize the audio stream ready for playing.
         */
        virtual void initStream() = 0;
};

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
    AudioSource *source;    // Source to get the audio from
    uint32_t pos;           // Playback position in samples. Needed for looping.
    float vol;              // Fractional playback volume (1.0 = normal volume)
    uint8_t flags;          // Various settings for the sample
    uint32_t loop_start;    // When looping go back to here
    uint32_t loop_end;      // Loop back when you get here, and play from here for aftertouch
};

#define MAX_SAMPLES 50


// Play the sample out of the left speaker
#define SMP_LEFT        0x01

// Play the sample out of the right speaker
#define SMP_RIGHT       0x02

// This sample is active
#define SMP_ACTIVE      0x04

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

        static void (*_hookPeak)(int32_t, int32_t);

	public:
		I2S(uint32_t sr);
		void begin();
        void end();
        void process();
        bool ready() { return ((!_bufferAFull) || (!_bufferBFull)); }
        bool fill(int16_t sample);
        bool fill(int16_t s1, int16_t s2);
        bool fillMono(int16_t sample);

        // New AudioSource based playing. No option for mono or stereo - the source dictates
        // the mode. You can choose to only play through one channel though.
        int play(AudioSource *source, float volume);
        int playLeft(AudioSource *source, float volume);
        int playRight(AudioSource *source, float volume);

        void stop(int s);
        void resume(int s);
        void cancel(int s);
        void setVolume(int s, float v);
        void enableLoop(int s, int st, int e, bool a);
        void disableLoop(int s);
        bool isPlaying(int s);
        void setSampleRate(uint32_t r);

        static void hookPeak(void (*f)(int32_t, int32_t)) { _hookPeak = f; }

};

#endif
