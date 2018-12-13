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
    DCH1ECONbits.SIRQEN = 1;
    DCH1ECONbits.CHSIRQ = _SPI2_TX_VECTOR;
    setIntVector(_DMA1_VECTOR, (isrFunc)I2S::DMA1ISR);
    setIntPriority(_DMA1_VECTOR, 6, 0);
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
    DCH2ECONbits.SIRQEN = 1;
    DCH2ECONbits.CHSIRQ = _SPI2_TX_VECTOR;
    setIntVector(_DMA2_VECTOR, (isrFunc)I2S::DMA2ISR);
    setIntPriority(_DMA2_VECTOR, 6, 0);
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
    if (doFillBuffer(_bufferA)) {
        _bufferAFull = 1;
    }
    //DCH2CONbits.CHEN = 1;
}

void __USER_ISR I2S::DMA2ISR(void) {
    DCH1CONbits.CHEN = 1;
    clearIntFlag(_DMA2_VECTOR);
    DCH2INTbits.CHSDIF = 0;
    _bufferBFull = 0;
    if (doFillBuffer(_bufferB)) {
        _bufferBFull = 1;
    }
    //DCH1CONbits.CHEN = 1;
}

bool I2S::doFillBuffer(int32_t *buf) {
//    int cnt = 0;
//    for (int i = 0; i < MAX_SAMPLES; i++) {
//        if (_samples[i].flags & SMP_ACTIVE) {
//            cnt++;
//            break;
//        }
//    }
//    if (cnt == 0) return false;

    int32_t peakLeft = 0;
    int32_t peakRight = 0;

    for (int sno = 0; sno < BSIZE/2; sno++) {
        int32_t left = 0;
        int32_t right = 0;
//        int playingl = 0;
//        int playingr = 0;

        for (int i = 0; i < MAX_SAMPLES; i++) {
            if ((_samples[i].flags & SMP_ACTIVE) && (_samples[i].flags & SMP_PLAYING)) {
                if (_samples[i].data != NULL) {
                    uint32_t p = (uint32_t)_samples[i].pos;
//                    float pct = _samples[i].pos - (int)_samples[i].pos;
                    if (_samples[i].flags & SMP_STEREO) {
                        p <<= 1; // Double it
                        if (_samples[i].flags & SMP_LEFT) {
//                            playingl++;
//                            float low = _samples[i].data[p] * (1.0-pct);
//                            float high = _samples[i].data[p+2] * pct;
//                            left = mix(left, (low + high) * _samples[i].vol);
                            left = mix(left, _samples[i].data[p] * _samples[i].vol);
                            if (abs(left) > peakLeft) peakLeft = abs(left);
                        }
                        if (_samples[i].flags & SMP_RIGHT) {
//                            playingr++;
//                            float low = _samples[i].data[p+1] * (1.0-pct);
//                            float high = _samples[i].data[p+3] * pct;
//                            right = mix(right, (low + high) * _samples[i].vol);
                            right = mix(right, _samples[i].data[p+1] * _samples[i].vol);
                            if (abs(right) > peakRight) peakRight = abs(right);
                        }
                    } else {
                        if (_samples[i].flags & SMP_LEFT) {
//                            playingl++;
//                            float low = _samples[i].data[p] * (1.0-pct);
//                            float high = _samples[i].data[p+1] * pct;
//                            left = mix(left, (low + high) * _samples[i].vol);
                            left = mix(left, _samples[i].data[p] * _samples[i].vol);
                            if (abs(left) > peakLeft) peakLeft = abs(left);
                        }
                        if (_samples[i].flags & SMP_RIGHT) {
//                            playingr++;
//                            float low = _samples[i].data[p] * (1.0-pct);
//                            float high = _samples[i].data[p+1] * pct;
//                            right = mix(right, (low + high) * _samples[i].vol);
                            right = mix(right, _samples[i].data[p] * _samples[i].vol);
                            if (abs(right) > peakRight) peakRight = abs(right);
                        }
                    }

                    _samples[i].pos += _samples[i].speed;

                    if (_samples[i].flags & SMP_LOOP) {
                        if (_samples[i].pos >= _samples[i].loop_end) {
                            _samples[i].pos = _samples[i].loop_start;
                        }
                    } else {
                        if (((uint32_t)_samples[i].pos) >= _samples[i].len) {
                            _samples[i].flags = 0;
                        }
                    }
                } else if (_samples[i].file != NULL) {
                    if (_samples[i].flags & SMP_STEREO) {
                        int16_t sl, sr;
                        uint32_t nrl = 0;
                        uint32_t nrr = 0;
                        _samples[i].file->fsread(&sl, 2, &nrl);
                        _samples[i].file->fsread(&sr, 2, &nrr);

                        if ((nrl != 2) || (nrr != 2)) {
                            if (_samples[i].flags & SMP_LOOP) {
                                _samples[i].file->fslseek(_samples[i].offset);
                            } else {
                                _samples[i].flags = 0;
                            }
                        } else {
                            if (_samples[i].flags & SMP_LEFT) {
//                                playingl++;
                                left = mix(left, sl * _samples[i].vol);
                                if (abs(left) > peakLeft) peakLeft = abs(left);
                            }
                            if (_samples[i].flags & SMP_RIGHT) {
//                                playingr++;
                                right = mix(right, sr * _samples[i].vol);
                                if (abs(right) > peakRight) peakRight = abs(right);
                            }
                        }
                    } else {
                        int16_t sm;
                        uint32_t nr = 0;
                        _samples[i].file->fsread(&sm, 2, &nr);

                        if (nr != 2) {
                            if (_samples[i].flags & SMP_LOOP) {
                                _samples[i].file->fslseek(_samples[i].offset);
                            } else {
                                _samples[i].flags = 0;
                            }
                        } else {
                            if (_samples[i].flags & SMP_LEFT) {
//                                playingl++;
                                left = mix(left, sm * _samples[i].vol);
                                if (abs(left) > peakLeft) peakLeft = abs(left);
                            }
                            if (_samples[i].flags & SMP_RIGHT) {
//                                playingr++;
                                right = mix(right, sm * _samples[i].vol);
                                if (abs(right) > peakRight) peakRight = abs(right);
                            }
                        }
                    }
                }
            }
        }
//        if (playingl == 0) {
//            buf[sno<<1] = 0;
//        } else {
//            left /= playingl;
            buf[sno<<1] = left << 16;
//        }
//        if (playingr == 0) {
//            buf[(sno<<1) + 1] = 0;
//        } else {
//            right /= playingr;
            buf[(sno<<1) + 1] = right << 16;
//        }
    }

    if (_hookPeak != NULL) {
        _hookPeak(peakLeft, peakRight);
    }
    return true;
}

int I2S::playStereo(const int16_t *data, uint32_t len, float vol, float speed, uint32_t offset) {
    for (int i = 0; i < MAX_SAMPLES; i++) {
//        uint32_t s = disableInterrupts();
        if ((_samples[i].flags & SMP_ACTIVE) == 0) {
            _samples[i].speed = speed;
            _samples[i].vol = vol;
            _samples[i].data = data;
            _samples[i].file = NULL;
            _samples[i].len = len;
            _samples[i].pos = offset;
            _samples[i].offset = offset;
            _samples[i].flags = SMP_ACTIVE | SMP_STEREO | SMP_LEFT | SMP_RIGHT | SMP_PLAYING;
//            restoreInterrupts(s);
            return i;
        }
//        restoreInterrupts(s);
    }
    return -1;
}

int I2S::playStereo(DFILE &file, float vol, float speed, uint32_t offset) {
    for (int i = 0; i < MAX_SAMPLES; i++) {
//        uint32_t s = disableInterrupts();
        if ((_samples[i].flags & SMP_ACTIVE) == 0) {
            _samples[i].speed = speed;
            _samples[i].vol = vol;
            _samples[i].data = NULL;
            _samples[i].file = &file;
            _samples[i].len = 0;
            _samples[i].pos = offset;
            _samples[i].offset = offset;
            _samples[i].flags = SMP_ACTIVE | SMP_STEREO | SMP_LEFT | SMP_RIGHT | SMP_PLAYING;
//            restoreInterrupts(s);
            return i;
        }
//        restoreInterrupts(s);
    }
    return -1;
}

int I2S::playMono(const int16_t *data, uint32_t len, float vol, float speed, uint32_t offset) {
    for (int i = 0; i < MAX_SAMPLES; i++) {
//        uint32_t s = disableInterrupts();
        if ((_samples[i].flags & SMP_ACTIVE) == 0) {
            _samples[i].speed = speed;
            _samples[i].vol = vol;
            _samples[i].data = data;
            _samples[i].file = NULL;
            _samples[i].len = len;
            _samples[i].pos = offset;
            _samples[i].offset = offset;
            _samples[i].flags = SMP_ACTIVE | SMP_LEFT | SMP_RIGHT | SMP_PLAYING;
//            restoreInterrupts(s);
            return i;
        }
//        restoreInterrupts(s);
    }
    return -1;
}

int I2S::playMono(DFILE &file, float vol, float speed, uint32_t offset) {
    for (int i = 0; i < MAX_SAMPLES; i++) {
//        uint32_t s = disableInterrupts();
        if ((_samples[i].flags & SMP_ACTIVE) == 0) {
            _samples[i].speed = speed;
            _samples[i].vol = vol;
            _samples[i].data = NULL;
            _samples[i].file = &file;
            _samples[i].len = 0;
            _samples[i].pos = offset;
            _samples[i].offset = offset;
            _samples[i].flags = SMP_ACTIVE | SMP_LEFT | SMP_RIGHT | SMP_PLAYING;
//            restoreInterrupts(s);
            return i;
        }
//        restoreInterrupts(s);
    }
    return -1;
}

int I2S::playMonoLeft(const int16_t *data, uint32_t len, float vol, float speed, uint32_t offset) {
    for (int i = 0; i < MAX_SAMPLES; i++) {
//        uint32_t s = disableInterrupts();
        if ((_samples[i].flags & SMP_ACTIVE) == 0) {
            _samples[i].speed = speed;
            _samples[i].vol = vol;
            _samples[i].data = data;
            _samples[i].file = NULL;
            _samples[i].len = len;
            _samples[i].pos = offset;
            _samples[i].offset = offset;
            _samples[i].flags = SMP_ACTIVE | SMP_LEFT | SMP_PLAYING;
//            restoreInterrupts(s);
            return i;
        }
//        restoreInterrupts(s);
    }
    return -1;
}

int I2S::playMonoLeft(DFILE &file, float vol, float speed, uint32_t offset) {
    for (int i = 0; i < MAX_SAMPLES; i++) {
//        uint32_t s = disableInterrupts();
        if ((_samples[i].flags & SMP_ACTIVE) == 0) {
            _samples[i].speed = speed;
            _samples[i].vol = vol;
            _samples[i].data = NULL;
            _samples[i].file = &file;
            _samples[i].len = 0;
            _samples[i].pos = offset;
            _samples[i].offset = offset;
            _samples[i].flags = SMP_ACTIVE | SMP_LEFT | SMP_PLAYING;
//            restoreInterrupts(s);
            return i;
        }
//        restoreInterrupts(s);
    }
    return -1;
}

int I2S::playMonoRight(const int16_t *data, uint32_t len, float vol, float speed, uint32_t offset) {
    for (int i = 0; i < MAX_SAMPLES; i++) {
//        uint32_t s = disableInterrupts();
        if ((_samples[i].flags & SMP_ACTIVE) == 0) {
            _samples[i].speed = speed;
            _samples[i].vol = vol;
            _samples[i].data = data;
            _samples[i].file = NULL;
            _samples[i].len = len;
            _samples[i].pos = offset;
            _samples[i].offset = offset;
            _samples[i].flags = SMP_ACTIVE | SMP_RIGHT | SMP_PLAYING;
//            restoreInterrupts(s);
            return i;
        }
//        restoreInterrupts(s);
    }
    return -1;
}

int I2S::playMonoRight(DFILE &file, float vol, float speed, uint32_t offset) {
    for (int i = 0; i < MAX_SAMPLES; i++) {
//        uint32_t s = disableInterrupts();
        if ((_samples[i].flags & SMP_ACTIVE) == 0) {
            _samples[i].speed = speed;
            _samples[i].vol = vol;
            _samples[i].data = NULL;
            _samples[i].file = &file;
            _samples[i].len = 0;
            _samples[i].pos = offset;
            _samples[i].offset = offset;
            _samples[i].flags = SMP_ACTIVE | SMP_RIGHT | SMP_PLAYING;
//            restoreInterrupts(s);
            return i;
        }
//        restoreInterrupts(s);
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

void I2S::setSpeed(int s, float v) {
    if (s >= MAX_SAMPLES) return;
    if (s < 0) return;
    _samples[s].speed = v;
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

