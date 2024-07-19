const float kFreqKnobMin = 20.f;
const float kFreqKnobMax = 20000.f;

class SinglePoleLowpassFilter {
private:
    float a1, b0;
    float y1;
    float sampleRate;
    float frequency;
    float dcBlock;
    const float dcBlockCoeff = 0.995f;

public:
    SinglePoleLowpassFilter() : y1(0), sampleRate(44100), frequency(1000), dcBlock(0) {
        updateCoefficients();
    }

    void setSampleRate(float sr) {
        sampleRate = sr;
        updateCoefficients();
    }

    void setCutoff(float freq) {
        frequency = freq;
        updateCoefficients();
    }

    void updateCoefficients() {
        float omega = 2.0f * M_PI * frequency / sampleRate;
        float g = std::tan(omega / 2.0f);
        b0 = g / (1.0f + g);
        a1 = (1.0f - g) / (1.0f + g);
    }

    float process(float input) {
        // AC coupling at input
        float acInput = input - dcBlock;
        dcBlock = dcBlock * dcBlockCoeff + input * (1.0f - dcBlockCoeff);

        // Apply Sallen-Key lowpass filter
        float output = b0 * acInput + b0 * y1 + a1 * y1;
        y1 = output;

        return output;
    }

    void reset() {
        y1 = 0;
        dcBlock = 0;
    }
};

class SinglePoleHighpassFilter {
private:
    float a1, b0, b1;
    float x1, y1;
    float sampleRate;
    float frequency;
    float dcBlock;
    const float dcBlockCoeff = 0.995f;

public:
    SinglePoleHighpassFilter() : x1(0), y1(0), sampleRate(44100), frequency(1000), dcBlock(0) {
        updateCoefficients();
    }

    void setSampleRate(float sr) {
        sampleRate = sr;
        updateCoefficients();
    }

    void setCutoff(float freq) {
        frequency = freq;
        updateCoefficients();
    }

    void updateCoefficients() {
        float omega = 2.0f * M_PI * frequency / sampleRate;
        float g = std::tan(omega / 2.0f);
        b0 = 1.0f / (1.0f + g);
        b1 = -b0;
        a1 = (1.0f - g) / (1.0f + g);
    }

    float process(float input) {
        // AC coupling at input
        float acInput = input - dcBlock;
        dcBlock = dcBlock * dcBlockCoeff + input * (1.0f - dcBlockCoeff);

        // Apply Sallen-Key highpass filter
        float output = b0 * acInput + b1 * x1 + a1 * y1;
        x1 = acInput;
        y1 = output;

        return output;
    }

    void reset() {
        x1 = 0;
        y1 = 0;
        dcBlock = 0;
    }
};

class TwoPole12dBBandpassFilter {
private:
    SinglePoleLowpassFilter lpf;
    SinglePoleHighpassFilter hpf;
    float sampleRate;
    float frequency;

public:
    TwoPole12dBBandpassFilter() : sampleRate(44100), frequency(1000) {
        updateCoefficients();
    }

    void setSampleRate(float sr) {
        sampleRate = sr;
        lpf.setSampleRate(sr);
        hpf.setSampleRate(sr);
        updateCoefficients();
    }

    void setFrequency(float freq) {
        frequency = freq;
        updateCoefficients();
    }

    void updateCoefficients() {
        lpf.setCutoff(frequency);
        hpf.setCutoff(frequency);
    }

    float process(float input) {
        float temp = lpf.process(input);
        return hpf.process(temp);
    }
};

class ThreePole18dBLowpassFilter {
private:
    SinglePoleLowpassFilter spf1, spf2, spf3;
    float sampleRate;
    float frequency;

public:
    ThreePole18dBLowpassFilter() : sampleRate(44100), frequency(1000) {
        updateCoefficients();
    }

    void setSampleRate(float sr) {
        sampleRate = sr;
        spf1.setSampleRate(sr);
        spf2.setSampleRate(sr);
        spf3.setSampleRate(sr);
        updateCoefficients();
    }

    void setFrequency(float freq) {
        frequency = freq;
        updateCoefficients();
    }

    void updateCoefficients() {
        spf1.setCutoff(frequency);
        spf2.setCutoff(frequency);
        spf3.setCutoff(frequency);
    }

    float process(float input) {
        float temp = spf1.process(input);
        temp = spf2.process(temp);
        return spf3.process(temp);
    }
};

class UnifiedFilter {
private:
    float sampleRate;
    float frequency;
    float resonance;
    float feedback;
    float lastOutput;
    float phase;
    float phaseIncrement;
    float outputGain;
    int oversampleFactor;

    SinglePoleLowpassFilter lpf1, lpf2, lpf3;
    SinglePoleHighpassFilter hpf;
    std::vector<double> oversampleBuffer;

    struct IIRFilter {
        double b0, b1, b2, a1, a2;
        double x1, x2, y1, y2;

        void setLowPass(int order, double cutoff, double q) {
            double w0 = 2 * M_PI * cutoff / 44100;
            double alpha = std::sin(w0) / (2 * q);

            b0 = (1 - std::cos(w0)) / 2;
            b1 = 1 - std::cos(w0);
            b2 = (1 - std::cos(w0)) / 2;
            a1 = -2 * std::cos(w0);
            a2 = 1 - alpha;

            double a0 = 1 + alpha;
            b0 /= a0; b1 /= a0; b2 /= a0; a1 /= a0; a2 /= a0;
        }

        double process(double input) {
            double output = b0 * input + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
            x2 = x1;
            x1 = input;
            y2 = y1;
            y1 = output;
            return output;
        }
    };

    IIRFilter antiAliasingFilter;

    float noise() {
        return (random::uniform() * 2.0f - 1.0f) * 0.05f;
    }

    float softClip(float x) {
        return std::tanh(x);
    }

    float lerp(float a, float b, float t) {
        return a + t * (b - a);
    }

    float generateSine() {
        float value = std::sin(phase);
        phase += phaseIncrement;
        if (phase >= 2 * M_PI) {
            phase -= 2 * M_PI;
        }
        return value;
    }

public:
    UnifiedFilter(int overSample = 16)
        : sampleRate(44100), frequency(1000), resonance(0), feedback(0), lastOutput(0),
          phase(0), phaseIncrement(0), outputGain(1.0f), oversampleFactor(overSample) {
        updateCoefficients();
        antiAliasingFilter.setLowPass(2, sampleRate * oversampleFactor / 2, 0.707);
        oversampleBuffer.resize(oversampleFactor);
    }

    void setSampleRate(float sr) {
        sampleRate = sr;
        lpf1.setSampleRate(sr * oversampleFactor);
        lpf2.setSampleRate(sr * oversampleFactor);
        lpf3.setSampleRate(sr * oversampleFactor);
        hpf.setSampleRate(sr * oversampleFactor);
        updateCoefficients();
        antiAliasingFilter.setLowPass(2, sampleRate * oversampleFactor / 2, 0.707);
    }

    void setFrequency(float freq) {
        frequency = freq;
        updateCoefficients();
    }

    void setResonance(float res) {
        resonance = res;
        updateCoefficients();
    }

    void setOutputGain(float gain) {
        outputGain = gain;
    }

    void updateCoefficients() {
        lpf1.setCutoff(frequency);
        lpf2.setCutoff(frequency);
        lpf3.setCutoff(frequency);
        hpf.setCutoff(frequency);
        feedback = resonance * (1.0 - 0.15 * (frequency / (sampleRate * 0.5)));
        phaseIncrement = 2.0f * M_PI * frequency / (sampleRate * oversampleFactor);
    }

    float process(float input, float modeCtrl) {
        double outputSum = 0.0;

        for (int i = 0; i < oversampleFactor; ++i) {
            double t = static_cast<double>(i) / oversampleFactor;
            double oversampledInput = lerp(input, lastOutput, t) + noise();
            double sine = generateSine();
            double resonanceInput = softClip(feedback * (lastOutput + sine * 0.1))*5;

            double lpOut = lpf3.process(lpf2.process(lpf1.process(oversampledInput + resonanceInput)));
            double bpOut = hpf.process(lpf2.process(lpf1.process(oversampledInput + resonanceInput)));
            double hpOut = hpf.process(oversampledInput + resonanceInput);

            double output;
            if (modeCtrl <= -5.0) {
                output = bpOut;
            } else if (modeCtrl >= 5.0) {
                output = hpOut;
            } else if (modeCtrl == 0.0) {
                output = lpOut;
            } else if (modeCtrl < 0.0) {
                double blendFactor = (modeCtrl + 5.0) / 5.0;
                output = bpOut * (1.0 - blendFactor) + lpOut * blendFactor;
            } else {
                double blendFactor = modeCtrl / 5.0;
                output = lpOut * (1.0 - blendFactor) + hpOut * blendFactor;
            }

            outputSum += antiAliasingFilter.process(output);
            lastOutput = output;
        }

        return (outputSum / oversampleFactor) * outputGain;
    }
};
