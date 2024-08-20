#include "plugin.hpp"

/*
 * Your sub-struct of Module contains all of the audio processing or generation code.
 *
 */


struct Monad : Module {
	enum ParamId {
		RANGE_PARAM,
		CFREQCTRL_PARAM,
		FFREQCTRL_PARAM,
		WIDTHCTRL_PARAM,
		FMCTRL_PARAM,
		SHAPECTRL_PARAM,
		PWNATT_PARAM,
		PARAMS_LEN
	};
	enum InputId {
		SYNCIN_INPUT,
		FREQCVIN_INPUT,
		EXTIN_INPUT,
		//EXTIN_INPUT,
		SHAPEIN_INPUT,
		PWMIN_INPUT,
		FMIN_INPUT,
		INPUTS_LEN
	};
	enum OutputId {
		NOISEOUT_OUTPUT,
		MORPHOUT_OUTPUT,
		SQROUT_OUTPUT,
		SINOUT_OUTPUT,
		TRIOUT_OUTPUT,
		SAWOUT_OUTPUT,
		OUTPUTS_LEN
	};
	enum LightId {
		LIGHTS_LEN
	};

	Monad() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		configParam(RANGE_PARAM, 0.f, 1.f, 0.f, "Frequency range");
		configParam(CFREQCTRL_PARAM, 0.f, 1.f, 0.f, "Coarse Frequency");
		configParam(FFREQCTRL_PARAM, 0.f, 1.f, 0.f, "Fine Frequency");
		configParam(WIDTHCTRL_PARAM, 0.f, 1.f, 0.f, "");
		configParam(FMCTRL_PARAM, -1.f, 1.f, 0.f, "Frequency Modulation");
		configParam(SHAPECTRL_PARAM, 0.f, 1.f, 0.f, "");
		configParam(PWNATT_PARAM, 0.f, 1.f, 0.f, "");
		configInput(SYNCIN_INPUT, "");
		configInput(FREQCVIN_INPUT, "");
		configInput(EXTIN_INPUT, "");
		configInput(EXTIN_INPUT, "");
		configInput(SHAPEIN_INPUT, "");
		configInput(PWMIN_INPUT, "");
		configInput(FMIN_INPUT, "");
		configOutput(NOISEOUT_OUTPUT, "");
		configOutput(MORPHOUT_OUTPUT, "");
		configOutput(SQROUT_OUTPUT, "");
		configOutput(SINOUT_OUTPUT, "Sinewave Out");
		configOutput(TRIOUT_OUTPUT, "");
		configOutput(SAWOUT_OUTPUT, "");
	}

/*
 * memeber variables specific to this module
 * 
 * _phase: keeps track of where oscillator is at in its cycle, used for sync as well
 * _gain: amplitude of our OutputId
 * _2PI: shorthand
 */

  void incrementPhase(float freq,float sampleRate) {
      
      
      float _phase = 0.f; 
      float _2PI = 2.f * M_PI;                   //


    //calculate the phase incrementPhase
      float phase_increment = _2PI * freq / sampleRate;

      //oscillator + 1 step
      _phase += phase_increment;
      if(_phase >= _2PI) {
        _phase -= _2PI;
      }
  };



	void process(const ProcessArgs& args) override {
	
    float _phase = 0.f;
    float _gain = 5.f; // for 10vpp -5v to 5v

    float freq = params[FFREQCTRL_PARAM].getValue() +
                params[CFREQCTRL_PARAM].getValue() +
                ( (inputs[FMIN_INPUT].getVoltage() / 5.f ) * params[FMCTRL_PARAM].getValue() ) +
                inputs[FREQCVIN_INPUT].getVoltage()  ; //whole cv is scaled to 0.2hz to 200hz

      if (RANGE_PARAM == 1.f){ //low freq RANGE_PARAM
        freq = freq * 0.01f;
      } else {
        freq = freq * 1.f; // is there a better way to do nothing?
      }

    float sine_out = _gain * sin(_phase);
    outputs[SINOUT_OUTPUT].setVoltage(sine_out); 
  
  

    incrementPhase (freq, args.sampleRate);
  
   }
};

 struct MonadWidget : ModuleWidget {
	MonadWidget(Monad* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/Monad.svg")));

		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		addParam(createParam<RoundBlackKnob>(mm2px(Vec(49.09, 22.79)), module, Monad::RANGE_PARAM));
		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(30.35, 25.95)), module, Monad::CFREQCTRL_PARAM));
		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(50.88, 40.825)), module, Monad::FFREQCTRL_PARAM));
		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(10.05, 58.95)), module, Monad::WIDTHCTRL_PARAM));
		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(30.35, 58.75)), module, Monad::FMCTRL_PARAM));
		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(50.8, 58.72)), module, Monad::SHAPECTRL_PARAM));
		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(10.05, 78.45)), module, Monad::PWNATT_PARAM));

		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.05, 18.95)), module, Monad::SYNCIN_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.05, 36.95)), module, Monad::FREQCVIN_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(23.981, 78.389)), module, Monad::EXTIN_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(37.106, 78.45)), module, Monad::EXTIN_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(51.05, 78.45)), module, Monad::SHAPEIN_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10.05, 95.95)), module, Monad::PWMIN_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(24.05, 95.95)), module, Monad::FMIN_INPUT));

		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(37.05, 95.95)), module, Monad::NOISEOUT_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(51.05, 95.95)), module, Monad::MORPHOUT_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(10.05, 110.95)), module, Monad::SQROUT_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(24.05, 110.95)), module, Monad::SINOUT_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(37.05, 110.95)), module, Monad::TRIOUT_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(51.05, 110.95)), module, Monad::SAWOUT_OUTPUT));
	  }
    
  };


Model* modelMonad = createModel<Monad, MonadWidget>("Monad");
