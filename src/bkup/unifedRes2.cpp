class UnifiedFilter {
private:
    SinglePoleLowpassFilter lpf1, lpf2, lpf3;
    SinglePoleHighpassFilter hpf;
    float sampleRate;
    float frequency;
    float resonance;
    float feedback;
    float lastOutput;

    // Simple random number generator for noise
    float noise() {
        return (random::uniform() * 2.0f - 1.0f) * 0.05f; // 50mV equivalent
    }

    // Soft clipping function to simulate diodes
    float softClip(float x) {
        return std::tanh(x);
    }

public:
    UnifiedFilter()
        : sampleRate(44100), frequency(1000), resonance(0), feedback(0), lastOutput(0) {
        updateCoefficients();
    }

    void setSampleRate(float sr) {
        sampleRate = sr;
        lpf1.setSampleRate(sr);
        lpf2.setSampleRate(sr);
        lpf3.setSampleRate(sr);
        hpf.setSampleRate(sr);
        updateCoefficients();
    }

    void setFrequency(float freq) {
        frequency = freq;
        updateCoefficients();
    }

    void setResonance(float res) {
        resonance = res;
        updateCoefficients();
    }

    void updateCoefficients() {
        lpf1.setCutoff(frequency);
        lpf2.setCutoff(frequency);
        lpf3.setCutoff(frequency);
        hpf.setCutoff(frequency);
        feedback = resonance * 4.0f; // Adjust this multiplier to taste
    }

    float process(float input, float modeCtrl) {
        float noisyInput = input + noise();
        float resonanceInput = softClip(feedback * lastOutput);

        // Process through all filters
        float lpOut = lpf3.process(lpf2.process(lpf1.process(noisyInput + resonanceInput)));
        float bpOut = hpf.process(lpf2.process(lpf1.process(noisyInput + resonanceInput)));
        float hpOut = hpf.process(noisyInput + resonanceInput);

        // Mix outputs based on mode control
        float output;
        if (modeCtrl <= -5.0f) {
            // Pure bandpass
            output = bpOut;
        } else if (modeCtrl >= 5.0f) {
            // Pure highpass
            output = hpOut;
        } else if (modeCtrl == 0.0f) {
            // Pure lowpass
            output = lpOut;
        } else if (modeCtrl < 0.0f) {
            // Blend between bandpass and lowpass
            float blendFactor = (modeCtrl + 5.0f) / 5.0f;
            output = bpOut * (1.0f - blendFactor) + lpOut * blendFactor;
        } else {
            // Blend between lowpass and highpass
            float blendFactor = modeCtrl / 5.0f;
            output = lpOut * (1.0f - blendFactor) + hpOut * blendFactor;
        }

        lastOutput = output;
        return output;
    }
};

class UnifiedFilter {
private:
    SinglePoleLowpassFilter lpf1, lpf2, lpf3;
    SinglePoleHighpassFilter hpf;
    float sampleRate;
    float frequency;
    float resonance;
    float feedback;
    float lastOutput;
    float phase;
    float phaseIncrement;

    // Simple random number generator for noise
    float noise() {
        return (random::uniform() * 2.0f - 1.0f) * 0.05f; // 50mV equivalent
    }

    // Soft clipping function to simulate diodes
    float softClip(float x) {
        return std::tanh(x);
    }

public:
    UnifiedFilter()
        : sampleRate(44100), frequency(1000), resonance(0), feedback(0), lastOutput(0), phase(0), phaseIncrement(0) {
        updateCoefficients();
    }

    void setSampleRate(float sr) {
        sampleRate = sr;
        lpf1.setSampleRate(sr);
        lpf2.setSampleRate(sr);
        lpf3.setSampleRate(sr);
        hpf.setSampleRate(sr);
        updateCoefficients();
    }

    void setFrequency(float freq) {
        frequency = freq;
        updateCoefficients();
    }

    void setResonance(float res) {
        resonance = res;
        updateCoefficients();
    }

    void updateCoefficients() {
        lpf1.setCutoff(frequency);
        lpf2.setCutoff(frequency);
        lpf3.setCutoff(frequency);
        hpf.setCutoff(frequency);
        feedback = resonance * 8.0f; // Reduced from 10.0f to 4.0f for more stable oscillation
        phaseIncrement = 2.0f * M_PI * frequency / sampleRate;
    }

    float process(float input, float modeCtrl) {
        // Generate a sine wave at the filter's frequency
        float sine = std::sin(phase);
        phase += phaseIncrement;
        if (phase >= 2.0f * M_PI) {
            phase -= 2.0f * M_PI;
        }

        float noisyInput = input + noise();
        float resonanceInput = softClip(feedback * (lastOutput + sine * 0.1f)); // Add some of the sine wave to the feedback path

        // Process through all filters
        float lpOut = lpf3.process(lpf2.process(lpf1.process(noisyInput + resonanceInput)));
        float bpOut = hpf.process(lpf2.process(lpf1.process(noisyInput + resonanceInput)));
        float hpOut = hpf.process(noisyInput + resonanceInput);

        // Mix outputs based on mode control
        float output;
        if (modeCtrl <= -5.0f) {
            // Pure bandpass
            output = bpOut;
        } else if (modeCtrl >= 5.0f) {
            // Pure highpass
            output = hpOut;
        } else if (modeCtrl == 0.0f) {
            // Pure lowpass
            output = lpOut;
        } else if (modeCtrl < 0.0f) {
            // Blend between bandpass and lowpass
            float blendFactor = (modeCtrl + 5.0f) / 5.0f;
            output = bpOut * (1.0f - blendFactor) + lpOut * blendFactor;
        } else {
            // Blend between lowpass and highpass
            float blendFactor = modeCtrl / 5.0f;
            output = lpOut * (1.0f - blendFactor) + hpOut * blendFactor;
        }

        lastOutput = output;
        return output;
    }
};
