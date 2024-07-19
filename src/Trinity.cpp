#include "plugin.hpp"
#include <cmath>

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
    SinglePoleLowpassFilter() : y1(0), sampleRate(88200), frequency(1000), dcBlock(0) {
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
    SinglePoleHighpassFilter() : x1(0), y1(0), sampleRate(88200), frequency(1000), dcBlock(0) {
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




class UnifiedFilter {
private:
    SinglePoleLowpassFilter lpf;
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
        : sampleRate(88200), frequency(1000), resonance(0), feedback(0), lastOutput(0) {
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

    void setResonance(float res) {
        resonance = res;
        updateCoefficients();
    }

    void updateCoefficients() {
        lpf.setCutoff(frequency);
        hpf.setCutoff(frequency);
        feedback = resonance * 4.0f; // Adjust this multiplier to taste
    }

    float process(float input, float modeCtrl) {
        float noisyInput = input + noise();
        float resonanceInput = softClip(feedback * (lastOutput*5));

        // Process through all filters
        float lpOut = lpf.process(lpf.process(lpf.process(noisyInput + resonanceInput)));
        float bpOut = hpf.process(lpf.process(noisyInput + resonanceInput));
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

struct Trinity : Module {
	enum ParamId {
		LEVEL_CTRL_PARAM,
		TYPE_PARAM,
		MODE_CTRL_PARAM,
		FREQ_CTRL_PARAM,
		FM_CTRL_PARAM,
		RES_CTRL_PARAM,
		ATT1_PARAM,
		ATT2_PARAM,
		ATT3_PARAM,
		PARAMS_LEN
	};
	enum InputId {
		LEVEL_CV_INPUT,
		IN1_INPUT,
		IN2_INPUT,
		IN3_INPUT,
		MODE_INPUT,
		FM_INPUT,
		FREQ_INPUT,
		RES_INPUT,
		INPUTS_LEN
	};
	enum OutputId {
		OUT_OUTPUT,
		OUTPUTS_LEN
	};
	enum LightId {
		LIGHTS_LEN
	};


  UnifiedFilter unifiedFilter;

	Trinity() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		configParam(LEVEL_CTRL_PARAM, 0.f, 1.f, 0.f, "VCA level");
		configParam(TYPE_PARAM, 0.0, 1.0, 1.0, "type");
		configParam(MODE_CTRL_PARAM, -5.0f, 5.0f, 0.f, "mode");
		configParam(FREQ_CTRL_PARAM, std::log2(kFreqKnobMin), std::log2(kFreqKnobMax), std::log2(kFreqKnobMax), "Frequency", " Hz", 2.f);
		configParam(FM_CTRL_PARAM, -1.f, 1.f, 0.f, "Frequency modulation", "%", 0, 100);
		configParam(RES_CTRL_PARAM, 0.f, 2.f, 0.f, "Resonance");
		configParam(ATT1_PARAM, 0.f, 2.f, 0.f, "drive 1");
		configParam(ATT2_PARAM, 0.f, 2.f, 0.f, "drive 2");
		configParam(ATT3_PARAM, 0.f, 2.f, 0.f, "drive 3");
		configInput(LEVEL_CV_INPUT, "level cv");
		configInput(IN1_INPUT, "Input 1");
		configInput(IN2_INPUT, "Input 2");
		configInput(IN3_INPUT, "Input 3");
		configInput(MODE_INPUT, "Mode CV");
		configInput(FM_INPUT, "FM CV");
		configInput(FREQ_INPUT, "Freq CV");
		configInput(RES_INPUT, "RESONANCE CV");
		configOutput(OUT_OUTPUT, "VCF/VCA OUTPUT");
		//highpassFilter.setSampleRate(44100.f);  // Initialize with a default sample rate
	}

//		FREQ_CTRL_PARAM, setup for a log response 20hz to 20khz



void process(const ProcessArgs& args) override {
     float inputValues[3];
     float summedInput = 0.0f;
     float filteredOutput = 0.0f;

     // Read and attenuate inputs
     for (int i = 0; i < 3; i++) {
         float input = inputs[IN1_INPUT + i].getVoltage();
         float gain = params[ATT1_PARAM + i].getValue();
         inputValues[i] = input * gain;
         summedInput += inputValues[i];
     }

     // Apply distortion to summed input
     if (summedInput > 5.0f) {
         float excess = summedInput - 5.0f;
         summedInput = 5.0f + std::tanh(excess);
     } else if (summedInput < -5.0f) {
         float excess = -5.0f - summedInput;
         summedInput = -5.0f - std::tanh(excess);
     }

     // Update filter parameters
     float cutoffFreq = std::pow(2.f, params[FREQ_CTRL_PARAM].getValue());

     // Apply frequency modulation if needed
     float fmAmount = params[FM_CTRL_PARAM].getValue();
     float fmCV = inputs[FM_INPUT].getVoltage() / 5.f; // Assuming +/-5V CV range
     cutoffFreq *= std::pow(2.f, fmAmount * fmCV);

     // Update unified filter parameters
     unifiedFilter.setSampleRate(args.sampleRate);
     unifiedFilter.setFrequency(cutoffFreq);

     float resonance = params[RES_CTRL_PARAM].getValue();//+inputs[RES_INPUT].getVoltage() / 1.f;
     unifiedFilter.setResonance(resonance);

     // Process unified filter
     float modeCtrl = params[MODE_CTRL_PARAM].getValue();
     filteredOutput = unifiedFilter.process(summedInput, modeCtrl);

     // Apply VCA gain and clamp
     float vcaGain = params[LEVEL_CTRL_PARAM].getValue();
     float vcaOut = rack::math::clamp(filteredOutput * vcaGain, -11.0f, 11.0f);

     // Output the processed signal
     outputs[OUT_OUTPUT].setVoltage(vcaOut);
 }
};

struct TrinityWidget : ModuleWidget {
	TrinityWidget(Trinity* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/Trinity.svg")));

		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
//
		addParam(createParamCentered<BefacoSlidePotBlack>(mm2px(Vec(50.839, 50.70)), module, Trinity::LEVEL_CTRL_PARAM));
		addParam(createParam<CKSS>(mm2px(Vec(21.602, 56.872)), module, Trinity::TYPE_PARAM));//+2.482,+2.482
		addParam(createParamCentered<Davies1900hBlackKnob>(mm2px(Vec(50.659, 18.616)), module, Trinity::MODE_CTRL_PARAM));
		addParam(createParamCentered<Davies1900hLargeBlackKnob>(mm2px(Vec(23.704, 26.077)), module, Trinity::FREQ_CTRL_PARAM));//21.222,23.594
		addParam(createParamCentered<Davies1900hBlackKnob>(mm2px(Vec(10.076, 58.667)), module, Trinity::FM_CTRL_PARAM));
		addParam(createParamCentered<Davies1900hBlackKnob>(mm2px(Vec(37.362, 58.667)), module, Trinity::RES_CTRL_PARAM));
		addParam(createParamCentered<Trimpot>(mm2px(Vec(10.246, 78.458)), module, Trinity::ATT1_PARAM));
		addParam(createParamCentered<Trimpot>(mm2px(Vec(23.698, 78.483)), module, Trinity::ATT2_PARAM));
		addParam(createParamCentered<Trimpot>(mm2px(Vec(37.266, 78.483)), module, Trinity::ATT3_PARAM));

		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(50.839, 78.483)), module, Trinity::LEVEL_CV_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.246, 95.635)), module, Trinity::IN1_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(23.698, 95.68)), module, Trinity::IN2_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(37.262, 95.68)), module, Trinity::IN3_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(50.828, 95.68)), module, Trinity::MODE_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.246, 110.623)), module, Trinity::FM_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(23.754, 110.623)), module, Trinity::FREQ_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(37.262, 110.623)), module, Trinity::RES_INPUT));

		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(50.89, 110.623)), module, Trinity::OUT_OUTPUT));
	}
};


Model* modelTrinity = createModel<Trinity, TrinityWidget>("Trinity");
