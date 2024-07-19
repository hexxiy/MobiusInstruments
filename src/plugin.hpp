#pragma once
#include <rack.hpp>


using namespace rack;

// Declare the Plugin, defined in plugin.cpp
extern Plugin* pluginInstance;

extern Model* modelTrinity;
// Declare each Model, defined in each module source file
// extern Model* modelMyModule;

 
struct BefacoSlidePotBlack : app::SvgSlider {
    BefacoSlidePotBlack() {
        setBackgroundSvg(Svg::load(asset::system("res/ComponentLibrary/BefacoSlidePot.svg")));
        setHandleSvg(Svg::load(asset::plugin(pluginInstance,"res/Components/BefacoSlidePotHandleSmallBlack.svg")));
        math::Vec margin = math::Vec(3.5, 3.5);
        setHandlePos(math::Vec(-1, 87).plus(margin), math::Vec(-1, -2).plus(margin));
        background->box.pos = margin;
        box.size = background->box.size.plus(margin.mult(2));
    }
};