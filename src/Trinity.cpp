/* to do 2024-08-12
check outputs with waveforms from scope pics

*/

#include "plugin.hpp"
#include <cmath>

const float kFreqKnobMin = 20.f;
const float kFreqKnobMax = 20000.f;
const float kDefaultSampleRate = 88200.f;
const float kDefaultFrequency = 1000.f;
const float kMaxResonance = 2.f;
const float kDefaultNegFbAmt = 0.25f;

class SinglePoleHighpassFilter {
private:
    float a1, b0, b1;
    float x1, y1;
    float sampleRate;
    float frequency;
    float negfb;

public:
    SinglePoleHighpassFilter() : x1(0), y1(0), sampleRate(kDefaultSampleRate ), frequency(kDefaultFrequency){
        updateCoefficients();
    }
    float noise() {
        return (random::uniform() * 2.0f - 1.0f) * 0.01f; // 10mV equivalent
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

        float acInput = input + noise() - negfb;

        float output = b0 * acInput + b1 * x1 + a1 * y1;
        x1 = acInput;
        y1 = output;
       // fbAMnt
        negfb = output;// * fbAmnt;
        return output;
    }
    void reset() {
        x1 = 0;
        y1 = 0;
        //dcBlock = 0;
    }
};

class SinglePoleLowpassFilter {
private:
    float a1, b0;
    float y1;
    float sampleRate;
    float frequency;
    float negfb;

public:
    SinglePoleLowpassFilter() : y1(0), sampleRate(kDefaultSampleRate), frequency(kDefaultFrequency){//, dcBlock(0) {
        updateCoefficients();
    }

      // Simple random number generator for noise
    float noise() {
        return (random::uniform() * 2.0f - 1.0f) * 0.01f; // 10mV equivalent
    }


    void setSampleRate(float sr) {
        sampleRate = sr;
        //hpf.setSampleRate(sampleRate);
        updateCoefficients();
    }

    void setCutoff(float freq) {
        frequency = freq;
        //hpf.setCutoff(33);
        updateCoefficients();
    }

    void updateCoefficients() {
        float omega = 2.0f * M_PI * frequency / sampleRate;
        float g = std::tan(omega / 2.0f);
        b0 = g / (1.0f + g);
        a1 = (1.0f - g) / (1.0f + g);
    }

    float process(float input) {

        float acInput = input + noise() - negfb;


        // Apply Sallen-Key lowpass filter
        float output = b0 * acInput + b0 * y1 + a1 * y1;
        y1 = output;

        negfb = output * 0.35f; //only a small amount of negative feedback
        return output * 1.0f;
    }

    void reset() {
        y1 = 0;

    }
};

class BiFilter {
private:
    float frequency;
    float sampleRate;
    float Q;
    float b0, b1, b2, a1, a2;
    float y1 = 0.0f, y2 = 0.0f;
    float negfb;

public:
    BiFilter() :  frequency(kDefaultFrequency),  sampleRate(kDefaultSampleRate), Q(kMaxResonance) {
        updateCoefficients();
    }

    //setter functions to allow filter to be dynamically changed

    void setQ(float q) {
        Q = q;
        //hpf.setSampleRate(sampleRate);
        updateCoefficients();
    }

    void setSampleRate(float sr) {
        sampleRate = sr;
        //hpf.setSampleRate(sampleRate);
        updateCoefficients();
    }

    void setCutoff(float freq) {
        frequency = freq;
        //hpf.setCutoff(33);
        updateCoefficients();
    }

    void updateCoefficients() {
        float omega = 2.0f * M_PI * frequency / sampleRate;
        float alpha = std::sin(omega) / (2.0f * Q);
        float cosw = std::cos(omega);

        float a0 = 1.0f + alpha;
        b0 = ((1.0f - cosw) / 2.0f) / a0;
        b1 = (1.0f - cosw) / a0;
        b2 = b0;
        a1 = (-2.0f * cosw) / a0;
        a2 = (1.0f - alpha) / a0;
    }

    float noise() {
        return (random::uniform() * 2.0f - 1.0f) * 0.02f; // 20mV equivalent
    }


    float process(float input) {
        float acInput = input + noise() - negfb;
        // Apply biquad lowpass filter
        float output = b0 * acInput + b1 * y1 + b2 * y2 - a1 * y1 - a2 * y2;
        y2 = y1;
        y1 = output;
        negfb = output * 0.35f;
        return output;
    }

};

class UnifiedFilter {
private:
    SinglePoleLowpassFilter lpf1, lpf2, lpf3, lpf4, lpf5;
    SinglePoleHighpassFilter hpf1, hpf2, hpf3;
    BiFilter bilpf;

    float sampleRate;
    float frequency;
    float resonance;
    float feedback;
    float lastOutput;
    float lastResonanceInput;


    // Soft clipping function to simulate diodes
    float softClip(float x) {
        return std::tanh(x);
    }

    float lpGain(float modeCtrl) {
        return 1.0f - abs(modeCtrl  / 5.0f );
    }

    float bpGain(float modeCtrl) {
      if (modeCtrl < 0) {
        return abs(-modeCtrl / 5.0f);
      }else{
        return 0;
      }
    }

    float hpGain(float modeCtrl) {
      if (modeCtrl > 0) {
        return modeCtrl / 5.0f;
      }else{
        return 0;
      }
    }

public:
    UnifiedFilter()
        : sampleRate(kDefaultSampleRate), frequency(kDefaultFrequency), resonance(0), feedback(0), lastOutput(0) {
        updateCoefficients();
    }

    float noise() {
        return (random::uniform() * 2.0f - 1.0f) * 0.04f; // 40mV equivalent
    }




    void setSampleRate(float sr) {
        sampleRate = sr;
        lpf1.setSampleRate(sr);
        lpf2.setSampleRate(sr);
        lpf3.setSampleRate(sr);
        lpf4.setSampleRate(sr);
        hpf1.setSampleRate(sr);
        hpf2.setSampleRate(sr);

        bilpf.setSampleRate(sr);

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

    void setQ(float res){
      float q = res;
      bilpf.setQ(q);
    }

    void updateCoefficients() {
        lpf1.setCutoff(frequency);
        lpf2.setCutoff(frequency);
        lpf3.setCutoff(frequency);
        lpf4.setCutoff(frequency);
        lpf5.setCutoff(frequency);
        hpf1.setCutoff(frequency);
        hpf2.setCutoff(frequency);
        hpf3.setCutoff(20.0f);

        bilpf.setCutoff(frequency);

        feedback = resonance * 4.0f; // Adjust this multiplier to taste
    }

    float process(float input1, float input2, float input3, float modeCtrl, float typeParam) {
        float summedInput = input1 + input2 + input3 + noise();
        float resonanceInput = softClip(feedback * (lastOutput));

        if (typeParam >= 1.0f) {  // Type 1 unity mode processing
            // 3-pole lowpass filter
            float lpOut = bilpf.process(lpf1.process(summedInput * lpGain(modeCtrl)));//lpf1.process(lpf2.process(lpf3.process(summedInput)) + resonanceInput);

            // Bandpass filter (HPF -> LPF)
            float bpOut = lpf4.process(hpf1.process(summedInput * bpGain(modeCtrl) + resonanceInput));

            // Highpass filter
            float hpOut = hpf2.process(summedInput * hpGain(modeCtrl));

            // resonance filter
            //float resOut = hpf3.process(lpf5.process(resonanceInput));

            // sum it al lup
            float output = lpOut  +
                           bpOut  +
                           hpOut;

            if (output > 5.0f) {
              float excess = output - 5.0f;
              output = 5.0f + std::tanh(excess);
              } else if (output < -5.0f) {
              float excess = -5.0f - output;
              output = -5.0f - std::tanh(excess);
            }

            lastOutput = output;
            return output;
        } else {  // Type 0 scanner mode processing

            // 3-pole lowpass filter
            float lpOut = lpf1.process(lpf2.process(lpf3.process(input1)));
            // Bandpass filter (HPF -> LPF)
            float bpOut = lpf4.process(hpf1.process(input2));
            // Highpass filter
            float hpOut = hpf2.process(input3);

            float summedOutput = lpOut +
                                 bpOut +
                                 hpOut;

            return summedOutput;
        }
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

  SinglePoleLowpassFilter singlePoleLowpassFilter;
  SinglePoleHighpassFilter singlePoleHighpassFilter;
  BiFilter bilpf;
  UnifiedFilter unifiedFilter;

	Trinity() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		configParam(LEVEL_CTRL_PARAM, 0.f, 1.f, 0.f, "VCA level");
		configParam(TYPE_PARAM, 0.0, 1.0, 1.0, "type");
		configParam(MODE_CTRL_PARAM, -5.0f, 5.0f, 0.f, "mode");
		configParam(FREQ_CTRL_PARAM, std::log2(kFreqKnobMin), std::log2(kFreqKnobMax), std::log2(kFreqKnobMax), "Frequency", " Hz", 2.f);
		configParam(FM_CTRL_PARAM, -1.f, 1.f, 0.f, "Frequency modulation", "%", 0, 100);
		configParam(RES_CTRL_PARAM, 0.f, 1.f, 0.f, "Resonance");
		configParam(ATT1_PARAM, 0.f, 1.5f, 0.f, "drive 1");
		configParam(ATT2_PARAM, 0.f, 1.5f, 0.f, "drive 2");
		configParam(ATT3_PARAM, 0.f, 1.5f, 0.f, "drive 3");
		configInput(LEVEL_CV_INPUT, "level cv");
		configInput(IN1_INPUT, "Input 1");
		configInput(IN2_INPUT, "Input 2");
		configInput(IN3_INPUT, "Input 3");
		configInput(MODE_INPUT, "Mode CV");
		configInput(FM_INPUT, "FM CV");
		configInput(FREQ_INPUT, "Freq CV");
		configInput(RES_INPUT, "RESONANCE CV");
		configOutput(OUT_OUTPUT, "VCF/VCA OUTPUT");
	}

void process(const ProcessArgs& args) override {
     float inputValues[3];
    // float summedInput = 0.0f;
     float filteredOutput = 0.0f;
     // Read and attenuate inputs
     for (int i = 0; i < 3; i++) {
         float input = inputs[IN1_INPUT + i].getVoltage();
         float gain = params[ATT1_PARAM + i].getValue();
         inputValues[i] = input * gain;

          if (inputValues[i] > 5.0f) {
            float excess = inputValues[i] - 5.0f;
            inputValues[i] = 5.0f + std::tanh(excess);
          } else if (inputValues[i] < -5.0f) {
            float excess = -5.0f - inputValues[i];
            inputValues[i] = -5.0f - std::tanh(excess);
          }

         //summedInput += inputValues[i];
     }
     // Update filter parameters
     float freqAmount = params[FREQ_CTRL_PARAM].getValue();
     float freqCV = inputs[FREQ_INPUT].getVoltage() / 5.f; // Assuming a 5V Range
     float cutoffFreq = std::pow(2.f, freqAmount + freqCV);
     // add freq CV, combine
     float fmAmount = params[FM_CTRL_PARAM].getValue();
     float fmCV = inputs[FM_INPUT].getVoltage() / 5.f; // Assuming +/-5V CV range
     cutoffFreq *= std::pow(2.f, fmAmount * fmCV);

     unifiedFilter.setSampleRate(args.sampleRate);
     unifiedFilter.setFrequency(cutoffFreq);

     float resonance = params[RES_CTRL_PARAM].getValue() + inputs[RES_INPUT].getVoltage() / 1.f;
     unifiedFilter.setResonance(resonance);
     // Process unified filter

     //sum up the mode cvs
     float modeCV = inputs[MODE_INPUT].getVoltage();
     float modeCtrl = rack::math::clamp(params[MODE_CTRL_PARAM].getValue() + modeCV, -5.0f,5.0f);

     float typeParam = params[TYPE_PARAM].getValue();

     filteredOutput = unifiedFilter.process(inputValues[0],inputValues[1],inputValues[2], modeCtrl,typeParam );
     // Apply VCA gain and clamp
     float vcaCV; 
                  
     bool cvinputPresent = inputs[LEVEL_CV_INPUT].isConnected();
     if (!cvinputPresent){
       vcaCV = 1.0f; // equivalent of normalled 5v input
     } else {
       vcaCV = inputs[LEVEL_CV_INPUT].getVoltage() / 5.0f;
     }

     float vcaGain = params[LEVEL_CTRL_PARAM].getValue() * vcaCV; //attenuate the level cv input
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
