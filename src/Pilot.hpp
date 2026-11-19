#pragma once
#include <Geode/Geode.hpp>

namespace pilot {
	// Runs once per physics tick from the GJBaseGameLayer::processCommands hook.
	void tick(GJBaseGameLayer* layer);
	// Clears held-input state and the reaction queue (called on level restart).
	void reset();
}
