#include <WavData.h>

void WavData::initData() {
    
    _pos = _data;
    _header = (wav_header *)_data;
    if (strncmp(_header->riff_header, "RIFF", 4) != 0) {
        Serial.println("Not RIFF");
        return; // Not RIFF file
    }
    if (strncmp(_header->wave_header, "WAVE", 4) != 0) {
        Serial.println("Not WAVE");
        return; // Not WAVE file
    }
    if (strncmp(_header->fmt_header, "fmt ", 4) != 0) {
        Serial.println("Bad format");
        return; // Corrupt header format
    }
    if (strncmp(_header->data_header, "data", 4) != 0) {
        Serial.println("Bad data");
        return; // Corrupt header data
    }

    _end = &_data[sizeof(_header) + _header->data_bytes];
    Serial.printf("Channels: %d\r\n", _header->num_channels);
    Serial.printf("Bit depth: %d\r\n", _header->bit_depth);
}

uint32_t WavData::getNextSampleBlock8(int16_t *buf, uint32_t samps) {
    for (int i = 0; i < samps; i++) {
        buf[i] = (*_pos++) << 8;
        if (_pos >= _end) {
            return i;
        }
    }
    return samps;
}

uint32_t WavData::getNextSampleBlock16(int16_t *buf, uint32_t samps) {
    for (int i = 0; i < samps; i++) {
        buf[i] = *_pos++;
        buf[i] |= (*_pos++) << 8;
        if (_pos >= _end) {
            return i;
        }
    }
    return samps;
}

void WavData::initStream() {
    initData();
}

uint32_t WavData::seekToFrame(uint32_t frame) {
    uint32_t sample = frame * _header->num_channels;
    uint8_t bps = 2;
    if (_header->bit_depth == 8) bps = 1;
    if (_header->bit_depth == 16) bps = 2;

    uint32_t startByte = sample * bps;
    _pos = &_data[startByte + sizeof(_header)];
    return frame;
}

uint32_t WavData::getNextSampleBlock(int16_t *buf, uint32_t nsamps) {
    if (_header->bit_depth == 8) return getNextSampleBlock8(buf, nsamps);
    if (_header->bit_depth == 16) return getNextSampleBlock16(buf, nsamps);
    return 0;
}

uint32_t WavData::getChannels() {
    return _header->num_channels;
}

uint32_t WavData::getFrameLength() {
    if (_header->bit_depth == 8) {
        return _header->data_bytes / _header->num_channels;
    }
    if (_header->bit_depth == 16) {
        return (_header->data_bytes / _header->num_channels) / 2;
    }
    return 0;
}
