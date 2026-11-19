#include <Geode/Geode.hpp>
#include <Geode/modify/GJBaseGameLayer.hpp>
#include <Geode/modify/PlayLayer.hpp>
#include <Geode/ui/Popup.hpp>
#include <Geode/utils/Keyboard.hpp>
#include "Pilot.hpp"

using namespace geode::prelude;

// ------------------------------------------------------------------ hooks

class $modify(PilotBGL, GJBaseGameLayer) {
	void processCommands(float dt, bool isHalfTick, bool isLastTick) {
		auto pl = PlayLayer::get();
		if (pl && static_cast<GJBaseGameLayer*>(pl) == this) {
			pilot::tick(this);
		}
		GJBaseGameLayer::processCommands(dt, isHalfTick, isLastTick);
	}
};

class $modify(PilotPlayLayer, PlayLayer) {
	void resetLevel() {
		PlayLayer::resetLevel();
		pilot::reset();
	}
};

// ------------------------------------------------------------------ menu

class PilotMenu : public geode::Popup {
protected:
	CCLabelBMFont* m_reactionLabel = nullptr;
	CCLabelBMFont* m_distanceLabel = nullptr;

	static constexpr float REACTION_MAX = 250.f;
	static constexpr float DIST_MIN = 150.f;
	static constexpr float DIST_MAX = 1200.f;

	bool init() {
		if (!Popup::init(320.f, 220.f)) return false;
		this->setTitle("GD Neural Pilot");

		auto* mod = Mod::get();

		// autopilot toggle
		auto* toggler = CCMenuItemToggler::createWithStandardSprites(
			this, menu_selector(PilotMenu::onToggle), 0.8f);
		toggler->toggle(mod->getSettingValue<bool>("autopilot"));
		m_buttonMenu->addChildAtPosition(toggler, Anchor::Center, {-120.f, 55.f});

		auto* onLabel = CCLabelBMFont::create("Autopilot", "bigFont.fnt");
		onLabel->setScale(0.6f);
		onLabel->setAnchorPoint({0.f, 0.5f});
		m_mainLayer->addChildAtPosition(onLabel, Anchor::Center, {-90.f, 55.f});

		// reaction time
		float reaction = static_cast<float>(mod->getSettingValue<double>("reaction-ms"));
		m_reactionLabel = CCLabelBMFont::create("", "goldFont.fnt");
		m_reactionLabel->setScale(0.6f);
		m_mainLayer->addChildAtPosition(m_reactionLabel, Anchor::Center, {0.f, 20.f});

		auto* reactionSlider = Slider::create(this, menu_selector(PilotMenu::onReaction), 0.7f);
		reactionSlider->setValue(reaction / REACTION_MAX);
		m_buttonMenu->addChildAtPosition(reactionSlider, Anchor::Center, {0.f, 0.f});

		// prediction distance
		float dist = static_cast<float>(mod->getSettingValue<double>("prediction-distance"));
		m_distanceLabel = CCLabelBMFont::create("", "goldFont.fnt");
		m_distanceLabel->setScale(0.6f);
		m_mainLayer->addChildAtPosition(m_distanceLabel, Anchor::Center, {0.f, -35.f});

		auto* distSlider = Slider::create(this, menu_selector(PilotMenu::onDistance), 0.7f);
		distSlider->setValue((dist - DIST_MIN) / (DIST_MAX - DIST_MIN));
		m_buttonMenu->addChildAtPosition(distSlider, Anchor::Center, {0.f, -55.f});

		auto* hint = CCLabelBMFont::create("F6 autopilot  -  F7 menu (rebind in mod settings)", "chatFont.fnt");
		hint->setScale(0.5f);
		m_mainLayer->addChildAtPosition(hint, Anchor::Bottom, {0.f, 14.f});

		this->refreshLabels(reaction, dist);
		return true;
	}

	void refreshLabels(float reaction, float dist) {
		m_reactionLabel->setString(fmt::format("Reaction time: {:.0f} ms", reaction).c_str());
		m_distanceLabel->setString(fmt::format("Prediction distance: {:.0f} units", dist).c_str());
	}

	void onToggle(CCObject* sender) {
		bool nowOn = !static_cast<CCMenuItemToggler*>(sender)->isToggled(); // callback fires before the flip
		Mod::get()->setSettingValue<bool>("autopilot", nowOn);
	}

	void onReaction(CCObject* sender) {
		float v = static_cast<SliderThumb*>(sender)->getValue();
		float ms = v * REACTION_MAX;
		Mod::get()->setSettingValue<double>("reaction-ms", ms);
		this->refreshLabels(ms, static_cast<float>(Mod::get()->getSettingValue<double>("prediction-distance")));
	}

	void onDistance(CCObject* sender) {
		float v = static_cast<SliderThumb*>(sender)->getValue();
		float d = DIST_MIN + v * (DIST_MAX - DIST_MIN);
		Mod::get()->setSettingValue<double>("prediction-distance", d);
		this->refreshLabels(static_cast<float>(Mod::get()->getSettingValue<double>("reaction-ms")), d);
	}

public:
	inline static PilotMenu* s_open = nullptr;

	static PilotMenu* create() {
		auto* ret = new PilotMenu();
		if (ret && ret->init()) {
			ret->autorelease();
			s_open = ret;
			return ret;
		}
		CC_SAFE_DELETE(ret);
		return nullptr;
	}

	~PilotMenu() {
		if (s_open == this) s_open = nullptr;
	}
};

// ------------------------------------------------------------------ hotkeys

static bool matches(std::string_view settingKey, KeyboardInputData const& data) {
	auto binds = Mod::get()->getSettingValue<std::vector<Keybind>>(settingKey);
	for (auto const& kb : binds) {
		if (kb.key == data.key && kb.modifiers == data.modifiers) return true;
	}
	return false;
}

$on_mod(Loaded) {
	KeyboardInputEvent().listen(+[](KeyboardInputData& data) {
		if (data.action != KeyboardInputData::Action::Press) return ListenerResult::Propagate;

		if (matches("menu-key", data)) {
			if (PilotMenu::s_open) {
				PilotMenu::s_open->onClose(nullptr);
			} else if (auto* menu = PilotMenu::create()) {
				menu->show();
			}
			return ListenerResult::Stop;
		}
		if (matches("toggle-key", data)) {
			auto* mod = Mod::get();
			mod->setSettingValue<bool>("autopilot", !mod->getSettingValue<bool>("autopilot"));
			return ListenerResult::Stop;
		}
		return ListenerResult::Propagate;
	}).leak();
}
