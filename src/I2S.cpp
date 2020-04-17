#include <I2S.h>

int32_t I2S::_bufferA[BSIZE];
int32_t I2S::_bufferB[BSIZE];
bool I2S::_bufferAFull = false;
bool I2S::_bufferBFull = false;
uint32_t I2S::_bufferPos = 0;
uint32_t I2S::_sampleRate = 0;

void (*I2S::_hookPeak)(int32_t, int32_t) = NULL;

sample I2S::_samples[MAX_SAMPLES];

I2S::I2S(uint32_t s) {
    _sampleRate = s;
}

void I2S::setSampleRate(uint32_t r) {
    _sampleRate = r;
}

void I2S::begin() {
    initClock();       
    initSPI();
    initDMA();  
}

void I2S::end() {
    uninitDMA();  
    uninitSPI();
    uninitClock();       
}

void I2S::uninitDMA() {
    DCH1CONbits.CHEN = 0;
    DCH2CONbits.CHEN = 0;
    clearIntEnable(_DMA1_VECTOR);
    clearIntEnable(_DMA2_VECTOR);
}

void I2S::initDMA() {
    DCH1INTbits.CHSDIE = 1;
    DCH1SSA = ((unsigned int)_bufferA) & 0x1FFFFFFF;
    DCH1DSA = ((unsigned int)&SPI2BUF) & 0x1FFFFFFF;
    DCH1SSIZ = BSIZE*4;
    DCH1DSIZ = 4;
    DCH1CSIZ = 4;
    DCH1CONbits.CHPRI = 0;
    DCH1ECONbits.SIRQEN = 1;
    DCH1ECONbits.CHSIRQ = _SPI2_TX_VECTOR;
    setIntVector(_DMA1_VECTOR, (isrFunc)I2S::DMA1ISR);
    setIntPriority(_DMA1_VECTOR, 1, 0);
    clearIntFlag(_DMA1_VECTOR);
    setIntEnable(_DMA1_VECTOR);
    DCH1CONbits.CHAEN = 0;
    DCH1CONbits.CHEN = 1;

    DCH2INTbits.CHSDIE = 1;
    DCH2SSA = ((unsigned int)_bufferB) & 0x1FFFFFFF;
    DCH2DSA = ((unsigned int)&SPI2BUF) & 0x1FFFFFFF;
    DCH2SSIZ = BSIZE*4;
    DCH2DSIZ = 4;
    DCH2CSIZ = 4;
    DCH2CONbits.CHPRI = 0;
    DCH2ECONbits.SIRQEN = 1;
    DCH2ECONbits.CHSIRQ = _SPI2_TX_VECTOR;
    setIntVector(_DMA2_VECTOR, (isrFunc)I2S::DMA2ISR);
    setIntPriority(_DMA2_VECTOR, 1, 0);
    clearIntFlag(_DMA2_VECTOR);
    setIntEnable(_DMA2_VECTOR);
    DCH2CONbits.CHAEN = 0;
    DCH2CONbits.CHEN = 0;

    DMACONbits.ON = 1;
}

void I2S::uninitClock() {
    REFO1CONbits.ON=0;
}

void I2S::initClock() {
    uint32_t refClock = _sampleRate * 256;
    uint32_t inputFrequency = getPeripheralClock(); ///2;
    uint32_t refDiv = inputFrequency / refClock;
    uint32_t refTrim = (((float)inputFrequency / (float)refClock) - refDiv) * 512.0;
    pinMode(PIN_MCLK, OUTPUT);
    mapPps(PIN_MCLK, PPS_OUT_REFCLK1);
    REFO1CONbits.ON=0;
    REFO1CONbits.ACTIVE=0;
    REFO1CONbits.OE=1;
    REFO1CONbits.ROSEL=0;
    REFO1CONbits.RODIV=refDiv; // 3
    REFO1TRIMbits.ROTRIM=refTrim; // 464;
    REFO1CONbits.ACTIVE=1;
    REFO1CONbits.ON=1;
}

void I2S::uninitSPI() {
    SPI2CONbits.ON = 0;
}

void I2S::initSPI() {
    pinMode(PIN_WCLK, OUTPUT);
    mapPps(PIN_WCLK, PPS_OUT_SS2);
    pinMode(PIN_DO, OUTPUT);
    mapPps(PIN_DO, PPS_OUT_SDO2);
    
    SPI2CONbits.ON = 0;
    SPI2CONbits.FRMEN = 0;
    SPI2CONbits.FRMSYNC = 0;
    SPI2CONbits.FRMPOL = 0;
    SPI2CONbits.MSSEN = 0;
    SPI2CONbits.FRMSYPW = 0;
    SPI2CONbits.FRMCNT = 0b000;
    SPI2CONbits.MCLKSEL = 1; // REFCLK
    SPI2CONbits.SPIFE = 0;
    SPI2CONbits.ENHBUF = 1;
    SPI2CONbits.SIDL = 0;
    SPI2CONbits.DISSDO = 0;
    SPI2CONbits.MODE32 = 1; // 32 bit per channel
    SPI2CONbits.MODE16 = 0; // 32 bit per channel
    SPI2CONbits.SMP = 0;
    SPI2CONbits.CKE = 1;
    SPI2CONbits.SSEN = 0;
    SPI2CONbits.CKP = 1;
    SPI2CONbits.MSTEN = 1; // Master mode
    SPI2CONbits.DISSDI = 1; // No SDI for now
    SPI2CONbits.STXISEL = 0b11;
    SPI2CONbits.SRXISEL = 0b00;

    SPI2CON2bits.SPISGNEXT = 0;
    SPI2CON2bits.FRMERREN = 0;
    SPI2CON2bits.SPIROVEN = 0;
    SPI2CON2bits.SPITUREN = 0;
    SPI2CON2bits.IGNROV = 0;
    SPI2CON2bits.IGNTUR = 1;
    SPI2CON2bits.AUDEN = 1; // Enable audio mode
    SPI2CON2bits.AUDMONO = 0; // Stereo audio
    SPI2CON2bits.AUDMOD = 0b00; // I2S mode

    SPI2BRG = 1;

    SPI2CONbits.ON = 1; // Turn it on
}

void __USER_ISR I2S::DMA1ISR(void) {
    DCH2CONbits.CHEN = 1;
    clearIntFlag(_DMA1_VECTOR);
    DCH1INTbits.CHSDIF = 0;
    _bufferAFull = 0;
//    if (doFillBuffer(_bufferA)) {
//        _bufferAFull = 1;
//    }
    //DCH2CONbits.CHEN = 1;
}

void __USER_ISR I2S::DMA2ISR(void) {
    DCH1CONbits.CHEN = 1;
    clearIntFlag(_DMA2_VECTOR);
    DCH2INTbits.CHSDIF = 0;
    _bufferBFull = 0;
//    if (doFillBuffer(_bufferB)) {
//        _bufferBFull = 1;
//    }
    //DCH1CONbits.CHEN = 1;
}

void I2S::process() {
    if (_bufferAFull == 0) {
        if(doFillBuffer(_bufferA)) {
            _bufferAFull = 1;
        }
    }
    if (_bufferBFull == 0) {
        if(doFillBuffer(_bufferB)) {
            _bufferBFull = 1;
        }
    }
}

bool I2S::doFillBuffer(int32_t *buf) {
    for (int i = 0; i < BSIZE; i++) {
        buf[i] = 0;
    }

    for (int smpNo = 0; smpNo < MAX_SAMPLES; smpNo++) {
        if (_samples[smpNo].source != NULL) {
            if ((_samples[smpNo].flags & SMP_ACTIVE) && (_samples[smpNo].flags & SMP_PLAYING)) {
                uint32_t chans = _samples[smpNo].source->getChannels();
                if (chans == 1) {
                    int16_t sampleSet[BSIZE/2];
                    uint32_t nsamples = _samples[smpNo].source->getNextSampleBlock(sampleSet, BSIZE/2);

                    _samples[smpNo].pos += nsamples;
                    if (_samples[smpNo].flags & SMP_LOOP) {
                        if (_samples[smpNo].pos >= _samples[smpNo].loop_end) {
                            int overflow = _samples[smpNo].pos - _samples[smpNo].loop_end;
                            nsamples -= overflow;
                        } 

                        _samples[smpNo].source->seekToFrame(_samples[smpNo].loop_start);
                        nsamples += _samples[smpNo].source->getNextSampleBlock(&sampleSet[nsamples], (BSIZE/2) - nsamples);
                    } else {
                        if (nsamples < BSIZE/2) {
                            _samples[smpNo].flags = 0; // Stop playing - we reached the end!
                        }
                    }

                    for (int i = 0; i < nsamples; i++) {
                        if (_samples[smpNo].flags & SMP_LEFT) {
                            buf[i << 1] = mix(buf[i << 1], sampleSet[i] * _samples[smpNo].vol);
                        }
                        if (_samples[smpNo].flags & SMP_RIGHT) {
                            buf[(i << 1) + 1] = mix(buf[(i << 1) + 1], sampleSet[i] * _samples[smpNo].vol);
                        }
                    }
                } else {
                    int16_t sampleSet[BSIZE];
                    uint32_t nsamples = _samples[smpNo].source->getNextSampleBlock(sampleSet, BSIZE);


                    _samples[smpNo].pos += nsamples/2;
                    if (_samples[smpNo].flags & SMP_LOOP) {
                        if (_samples[smpNo].pos >= _samples[smpNo].loop_end) {
                            int overflow = _samples[smpNo].pos - _samples[smpNo].loop_end;
                            nsamples -= overflow;
                        } 

                        _samples[smpNo].source->seekToFrame(_samples[smpNo].loop_start);
                        nsamples += _samples[smpNo].source->getNextSampleBlock(&sampleSet[nsamples], (BSIZE) - nsamples);
                    } else {
                        if (nsamples < BSIZE) {
                            _samples[smpNo].flags = 0; // Stop playing - we reached the end!
                        }
                    }

                    for (int i = 0; i < nsamples; i++) {
                        if ((i & 0x01) == 0) { // Left sample
                            if (_samples[smpNo].flags & SMP_LEFT) {
                                buf[i] = mix(buf[i], sampleSet[i] * _samples[smpNo].vol);
                            }
                        } else {
                            if (_samples[smpNo].flags & SMP_RIGHT) {
                                buf[i] = mix(buf[i], sampleSet[i] * _samples[smpNo].vol);
                            }
                        }
                    }
                }
            }
        }
    }


    int32_t peakLeft = 0;
    int32_t peakRight = 0;
    for (int i = 0; i < BSIZE/2; i++) {

        int32_t l = abs(buf[i << 1]);
        int32_t r = abs(buf[(i << 1) + 1]);
        if (l > peakLeft) peakLeft = l;
        if (r > peakRight) peakRight = r;

    }
    if (_hookPeak != NULL) {
        _hookPeak(peakLeft, peakRight);
    }
    for (int i = 0; i < BSIZE; i++) {
        buf[i] <<= 16;
    }

    return true;
}

int I2S::play(AudioSource *s, float vol) {
    for (int i = 0; i < MAX_SAMPLES; i++) {
        if ((_samples[i].flags & SMP_ACTIVE) == 0) {
            s->initStream();
            s->seekToFrame(0);
            _samples[i].source = s;
            _samples[i].vol = vol;
            _samples[i].pos = 0;
            _samples[i].flags = SMP_ACTIVE | SMP_RIGHT | SMP_LEFT | SMP_PLAYING;
            return i;
        }
    }
    return -1;
}

int I2S::playLeft(AudioSource *s, float vol) {
    for (int i = 0; i < MAX_SAMPLES; i++) {
        if ((_samples[i].flags & SMP_ACTIVE) == 0) {
            s->initStream();
            s->seekToFrame(0);
            _samples[i].source = s;
            _samples[i].vol = vol;
            _samples[i].pos = 0;
            _samples[i].flags = SMP_ACTIVE | SMP_LEFT | SMP_PLAYING;
            return i;
        }
    }
    return -1;
}

int I2S::playRight(AudioSource *s, float vol) {
    for (int i = 0; i < MAX_SAMPLES; i++) {
        if ((_samples[i].flags & SMP_ACTIVE) == 0) {
            s->initStream();
            s->seekToFrame(0);
            _samples[i].source = s;
            _samples[i].vol = vol;
            _samples[i].pos = 0;
            _samples[i].flags = SMP_ACTIVE | SMP_RIGHT | SMP_PLAYING;
            return i;
        }
    }
    return -1;
}

void I2S::cancel(int s) {
    if (s >= MAX_SAMPLES) return;
    if (s < 0) return;
    _samples[s].flags = 0;
}

void I2S::stop(int s) {
    if (s >= MAX_SAMPLES) return;
    if (s < 0) return;
    if (_samples[s].flags & SMP_AFTERTOUCH) {
        _samples[s].pos = _samples[s].loop_end;
        _samples[s].flags &= ~SMP_LOOP;
    } else {
        _samples[s].flags &= ~SMP_PLAYING;
    }
}

void I2S::resume(int s) {
    if (s >= MAX_SAMPLES) return;
    if (s < 0) return;
    _samples[s].flags |= SMP_PLAYING;
}

void I2S::setVolume(int s, float v) {
    if (s >= MAX_SAMPLES) return;
    if (s < 0) return;
    _samples[s].vol = v;
}

void I2S::enableLoop(int s, int st, int e, bool a) {
    if (s >= MAX_SAMPLES) return;
    if (s < 0) return;
    _samples[s].flags |= SMP_LOOP;
    _samples[s].loop_start = st;
    _samples[s].loop_end = e;
    if (a) _samples[s].flags |= SMP_AFTERTOUCH;
}

void I2S::disableLoop(int s) {
    if (s >= MAX_SAMPLES) return;
    if (s < 0) return;
    _samples[s].flags &= ~SMP_LOOP;
}

bool I2S::isPlaying(int s) {
    if (s >= MAX_SAMPLES) return false;
    if (s < 0) return false;
    return _samples[s].flags & SMP_PLAYING;
}

bool I2S::fillMono(int16_t s) {
    if (!_bufferAFull) {
        _bufferA[_bufferPos++] = s << 16;
        _bufferA[_bufferPos++] = s << 16;
        if (_bufferPos >= BSIZE) {
            _bufferPos = 0;
            _bufferAFull = true;
        }
        return true;
    }
    if (!_bufferBFull) {
        _bufferB[_bufferPos++] = s << 16;
        _bufferB[_bufferPos++] = s << 16;
        if (_bufferPos >= BSIZE) {
            _bufferPos = 0;
            _bufferBFull = true;
        }
        return true;
    }
    return false;
}

bool I2S::fill(int16_t s1, int16_t s2) {
    if (!_bufferAFull) {
        _bufferA[_bufferPos++] = s1 << 16;
        _bufferA[_bufferPos++] = s2 << 16;
        if (_bufferPos == BSIZE) {
            _bufferPos = 0;
            _bufferAFull = true;
        }
        return true;
    }
    if (!_bufferBFull) {
        _bufferB[_bufferPos++] = s1 << 16;
        _bufferB[_bufferPos++] = s2 << 16;
        if (_bufferPos == BSIZE) {
            _bufferPos = 0;
            _bufferBFull = true;
        }
        return true;
    }
    return false;
}
bool I2S::fill(int16_t s) {
    if (!_bufferAFull) {
        _bufferA[_bufferPos++] = s << 16;
        if (_bufferPos == BSIZE) {
            _bufferPos = 0;
            _bufferAFull = true;
        }
        return true;
    }
    if (!_bufferBFull) {
        _bufferB[_bufferPos++] = s << 16;
        if (_bufferPos == BSIZE) {
            _bufferPos = 0;
            _bufferBFull = true;
        }
        return true;
    }
    return false;
}

int16_t I2S::mix(int16_t a, int16_t b)
{
    int z;
    unsigned int fa, fb, fz;
    fa = a + 32768;
    fb = b + 32768;

    if (fa < 32768 && fb < 32768) {
        fz = (fa * fb) / 32768;
    } else {
        fz = (2 * (fa + fb)) - ((fa * fb) / 32768) - 65536;
    }

    z = fz - 32768;
    return z;
}

