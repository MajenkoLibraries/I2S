#include <I2S.h>
#include <DFATFS.h>
#include <WavFile.h>

bool WavFile::initFile() {
    _file->fslseek(0);
    uint32_t nr;
    if(_file->fsread((uint8_t *)&_header, sizeof(_header), &nr) != FR_OK) {
        Serial.println("Unable to read");
        return false;
    }
    if (strncmp(_header.riff_header, "RIFF", 4) != 0) {
        Serial.println("Not RIFF");
        return false; // Not RIFF file
    }
    if (strncmp(_header.wave_header, "WAVE", 4) != 0) {
        Serial.println("Not WAVE");
        return false; // Not WAVE file
    }
    if (strncmp(_header.fmt_header, "fmt ", 4) != 0) {
        Serial.println("Bad format");
        return false; // Corrupt header format
    }
    if (strncmp(_header.data_header, "data", 4) != 0) {
        Serial.println("Bad data");
        return false; // Corrupt header data
    }

    Serial.printf("Channels: %d\r\n", _header.num_channels);
    Serial.printf("Bit depth: %d\r\n", _header.bit_depth);
    return true;
}

uint32_t WavFile::getNextSampleBlock8(int16_t *buf, uint32_t samps) {
    uint8_t ibuf[samps];
    uint32_t nr = 0;
    if (_file->fsread(ibuf, samps, &nr) == FR_OK) {
        for (int i = 0; i < nr; i++) {
            buf[i] = ibuf[i] << 8;
        }
        return nr;
    }
    return 0;
}

uint32_t WavFile::getNextSampleBlock16(int16_t *buf, uint32_t samps) {
    uint32_t nr = 0;
    int fr;
    if ((fr = _file->fsread((uint8_t *)buf, samps * 2, &nr, samps/256 + 2)) == FR_OK) {
        return nr / 2;
    }
    return 0;
}

void WavFile::initStream() {
    initFile();
}

uint32_t WavFile::seekToFrame(uint32_t frame) {
    uint32_t sample = frame * _header.num_channels;
    uint8_t bps = 2;
    if (_header.bit_depth == 8) bps = 1;
    if (_header.bit_depth == 16) bps = 2;


    uint32_t startByte = sample * bps;
    Serial.printf("Seek %d = %d\r\n", frame, startByte + sizeof(_header));
    if (_file->fslseek(startByte + sizeof(_header)) != FR_OK) {
        return 0;
    }
    return frame;
}

uint32_t WavFile::getNextSampleBlock(int16_t *buf, uint32_t nsamps) {
    if (_header.bit_depth == 8) return getNextSampleBlock8(buf, nsamps);
    if (_header.bit_depth == 16) return getNextSampleBlock16(buf, nsamps);
    return 0;
}

uint32_t WavFile::getChannels() {
    return _header.num_channels;
}

uint32_t WavFile::getFrameLength() {
    if (_header.bit_depth == 8) {
        return _header.data_bytes / _header.num_channels;
    }
    if (_header.bit_depth == 16) {
        return (_header.data_bytes / _header.num_channels) / 2;
    }
    return 0;
}
