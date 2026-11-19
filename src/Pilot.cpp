// GD Neural Pilot - look-ahead autoplayer.
//
// Not a neural net: it is a forward physics simulator. Every tick it simulates
// the player a few hundred steps ahead with and without a button press and
// presses only when "no press" dies and "press now" survives.
//
// Cube / robot : full simulation (jumps, pads, orbs, gravity portals, blocks, spikes, saws)
// Ship / UFO / swing / wave : steers toward the middle of the nearest free vertical gap
// Ball / spider : flips when a hazard enters its lane
//
// Every number in `phys` is an approximation of GD's physics. If the bot jumps
// early/late, tune them there first.

#include "Pilot.hpp"
#include <algorithm>
#include <cmath>
#include <deque>
#include <vector>

using namespace geode::prelude;

namespace pilot {

namespace phys {
	constexpr float STEP       = 1.f / 240.f;
	constexpr float GRAVITY    = 0.958199f * 3600.f;   // u/s^2 (cube)
	constexpr float CUBE_JUMP  = 11.180032f * 60.f;    // u/s
	constexpr float PAD_YELLOW = 16.0f * 60.f;
	constexpr float PAD_PINK   = 10.4f * 60.f;
	constexpr float PAD_RED    = 20.0f * 60.f;
	constexpr float ORB_YELLOW = 11.18f * 60.f;
	constexpr float ORB_PINK   = 7.14f * 60.f;
	constexpr float ORB_RED    = 14.0f * 60.f;
	constexpr float GROUND_Y   = 105.f;                // top of the floor
	constexpr float CEIL_Y     = 405.f;                // fallback ceiling when gravity is flipped
	constexpr float HAZARD_SHRINK = 0.7f;              // hazard AABB -> real hitbox
	constexpr float INNER      = 0.4f;                 // player's hazard hitbox / full hitbox
	constexpr float MARGIN     = 4.f;                  // safety padding on hazards (units)
	constexpr float LAND_TOL   = 8.f;                  // how far into a block top still counts as landing
}

enum class Kind { None, Solid, Hazard, Pad, Ring, GravPortal, ModePortal };

struct Obj {
	float x0, y0, x1, y1;
	GameObjectType type;
	Kind kind;
};

struct Body {
	float x, y, vy;     // vy in units/s, +up
	float g;            // +1 normal, -1 flipped
	float speed;        // units/s
	float hw, hh;       // half extents
	bool ground;
};

// ---------------------------------------------------------------- state

static std::vector<Obj> s_objs;           // sorted by x0
static GJBaseGameLayer* s_owner = nullptr;
static size_t s_objCount = 0;
static bool s_held = false;
static bool s_lastWant = false;
static double s_lastFlip = -10.0;
static double s_lastTap = -10.0;
static std::deque<std::pair<double, bool>> s_queue;

struct Config { bool on = false; float reactionMs = 0.f; float lookahead = 480.f; };
static Config s_cfg;
static int s_cfgAge = 1000;

static void refreshConfig() {
	if (++s_cfgAge < 20) return;
	s_cfgAge = 0;
	auto* m = Mod::get();
	s_cfg.on = m->getSettingValue<bool>("autopilot");
	s_cfg.reactionMs = static_cast<float>(m->getSettingValue<double>("reaction-ms"));
	s_cfg.lookahead = static_cast<float>(m->getSettingValue<double>("prediction-distance"));
}

void reset() {
	s_held = false;
	s_lastWant = false;
	s_lastFlip = -10.0;
	s_lastTap = -10.0;
	s_queue.clear();
}

// ---------------------------------------------------------------- level scan

static Kind kindOf(GameObjectType t) {
	switch (t) {
		case GameObjectType::Solid:
			return Kind::Solid;
		case GameObjectType::Hazard:
			return Kind::Hazard;
		case GameObjectType::YellowJumpPad:
		case GameObjectType::PinkJumpPad:
		case GameObjectType::RedJumpPad:
		case GameObjectType::GravityPad:
			return Kind::Pad;
		case GameObjectType::YellowJumpRing:
		case GameObjectType::PinkJumpRing:
		case GameObjectType::RedJumpRing:
		case GameObjectType::GravityRing:
		case GameObjectType::GreenRing:
			return Kind::Ring;
		case GameObjectType::InverseGravityPortal:
		case GameObjectType::NormalGravityPortal:
			return Kind::GravPortal;
		case GameObjectType::ShipPortal:
		case GameObjectType::CubePortal:
		case GameObjectType::BallPortal:
		case GameObjectType::UfoPortal:
		case GameObjectType::WavePortal:
		case GameObjectType::RobotPortal:
		case GameObjectType::SpiderPortal:
		case GameObjectType::SwingPortal:
			return Kind::ModePortal;
		default:
			return Kind::None; // slopes, decorations, triggers, coins... ignored
	}
}

static void rebuild(GJBaseGameLayer* l) {
	s_objs.clear();
	s_owner = l;
	s_objCount = l->m_objects ? l->m_objects->count() : 0;
	if (!l->m_objects) return;

	for (auto* o : CCArrayExt<GameObject*>(l->m_objects)) {
		if (!o || o->m_isTrigger || o->m_isNoTouch) continue;
		Kind k = kindOf(o->m_objectType);
		if (k == Kind::None) continue;

		CCRect const& r = o->getObjectRect();
		Obj ob{r.getMinX(), r.getMinY(), r.getMaxX(), r.getMaxY(), o->m_objectType, k};

		if (k == Kind::Hazard) {
			float cx = (ob.x0 + ob.x1) * 0.5f, cy = (ob.y0 + ob.y1) * 0.5f;
			float hx = (ob.x1 - ob.x0) * 0.5f * phys::HAZARD_SHRINK;
			float hy = (ob.y1 - ob.y0) * 0.5f * phys::HAZARD_SHRINK;
			if (o->m_objectRadius > 0.f) { // saws and other round hazards
				hx = hy = o->m_objectRadius * 0.8f;
			}
			ob.x0 = cx - hx; ob.x1 = cx + hx; ob.y0 = cy - hy; ob.y1 = cy + hy;
		}
		s_objs.push_back(ob);
	}
	std::sort(s_objs.begin(), s_objs.end(), [](Obj const& a, Obj const& b) { return a.x0 < b.x0; });
}

static void gather(float xMin, float xMax, std::vector<Obj>& out) {
	out.clear();
	constexpr float MAX_WIDTH = 200.f; // widest object we expect (sort key is x0)
	auto it = std::lower_bound(s_objs.begin(), s_objs.end(), xMin - MAX_WIDTH,
		[](Obj const& o, float v) { return o.x0 < v; });
	for (; it != s_objs.end() && it->x0 <= xMax; ++it) {
		if (it->x1 >= xMin) out.push_back(*it);
	}
}

// ---------------------------------------------------------------- simulation

static bool hit(float ax0, float ay0, float ax1, float ay1, Obj const& o, float grow = 0.f) {
	return ax0 < o.x1 + grow && ax1 > o.x0 - grow && ay0 < o.y1 + grow && ay1 > o.y0 - grow;
}

static void applyPad(Body& b, GameObjectType t) {
	switch (t) {
		case GameObjectType::YellowJumpPad: b.vy = phys::PAD_YELLOW * b.g; break;
		case GameObjectType::PinkJumpPad:   b.vy = phys::PAD_PINK * b.g; break;
		case GameObjectType::RedJumpPad:    b.vy = phys::PAD_RED * b.g; break;
		case GameObjectType::GravityPad:    b.g = -b.g; b.vy = 0.f; break;
		default: break;
	}
	b.ground = false;
}

static void applyRing(Body& b, GameObjectType t) {
	switch (t) {
		case GameObjectType::YellowJumpRing: b.vy = phys::ORB_YELLOW * b.g; break;
		case GameObjectType::PinkJumpRing:   b.vy = phys::ORB_PINK * b.g; break;
		case GameObjectType::RedJumpRing:    b.vy = phys::ORB_RED * b.g; break;
		case GameObjectType::GravityRing:    b.g = -b.g; b.vy = 0.f; break;
		case GameObjectType::GreenRing:      b.g = -b.g; b.vy = phys::ORB_YELLOW * b.g * 0.7f; break;
		default: break;
	}
	b.ground = false;
}

// Returns true if the body survives `reach` units of travel.
// press0: button is down on the first step. autoTap: later orb overlaps are tapped automatically.
static bool simulate(Body b, bool press0, bool autoTap, std::vector<Obj> const& w, float reach) {
	using namespace phys;
	int steps = static_cast<int>(reach / std::max(b.speed, 1.f) / STEP);
	steps = std::clamp(steps, 1, 900);
	std::vector<char> used(w.size(), 0);

	for (int i = 0; i < steps; ++i) {
		bool press = (i == 0) && press0;
		if (press && b.ground) {
			b.vy = CUBE_JUMP * b.g;
			b.ground = false;
			press = false;
		}

		// pads, rings, portals (full-size box)
		for (size_t k = 0; k < w.size(); ++k) {
			if (used[k]) continue;
			Obj const& o = w[k];
			if (!hit(b.x - b.hw, b.y - b.hh, b.x + b.hw, b.y + b.hh, o)) continue;
			switch (o.kind) {
				case Kind::Pad: used[k] = 1; applyPad(b, o.type); break;
				case Kind::Ring:
					if (press || (autoTap && i > 0)) { used[k] = 1; applyRing(b, o.type); press = false; }
					break;
				case Kind::GravPortal:
					used[k] = 1;
					b.g = (o.type == GameObjectType::InverseGravityPortal) ? -1.f : 1.f;
					b.ground = false;
					break;
				case Kind::ModePortal:
					return true; // can't model other vehicles: assume fine
				default: break;
			}
		}

		float prevY = b.y;
		if (!b.ground) b.vy -= GRAVITY * b.g * STEP;
		b.y += b.vy * STEP;
		b.x += b.speed * STEP;

		bool landed = false;
		if (b.g > 0.f && b.y - b.hh <= GROUND_Y) { b.y = GROUND_Y + b.hh; b.vy = 0.f; landed = true; }
		if (b.g < 0.f && b.y + b.hh >= CEIL_Y)   { b.y = CEIL_Y - b.hh;  b.vy = 0.f; landed = true; }

		// solids: land on them or die
		for (auto const& o : w) {
			if (o.kind != Kind::Solid) continue;
			if (!hit(b.x - b.hw, b.y - b.hh, b.x + b.hw, b.y + b.hh, o)) continue;
			if (b.g > 0.f) {
				if (prevY - b.hh >= o.y1 - LAND_TOL && b.vy <= 0.f) { b.y = o.y1 + b.hh; b.vy = 0.f; landed = true; }
				else return false;
			} else {
				if (prevY + b.hh <= o.y0 + LAND_TOL && b.vy >= 0.f) { b.y = o.y0 - b.hh; b.vy = 0.f; landed = true; }
				else return false;
			}
		}

		if (landed) {
			b.ground = true;
		} else if (b.ground) {
			bool supported = (b.g > 0.f) ? (b.y - b.hh <= GROUND_Y + 0.6f) : (b.y + b.hh >= CEIL_Y - 0.6f);
			for (auto const& o : w) {
				if (supported) break;
				if (o.kind != Kind::Solid) continue;
				if (o.x0 >= b.x + b.hw || o.x1 <= b.x - b.hw) continue;
				float gap = (b.g > 0.f) ? std::fabs((b.y - b.hh) - o.y1) : std::fabs((b.y + b.hh) - o.y0);
				if (gap < 1.5f) supported = true;
			}
			if (!supported) b.ground = false;
		}

		// hazards use the small inner hitbox
		float ix = b.hw * INNER, iy = b.hh * INNER;
		for (auto const& o : w) {
			if (o.kind == Kind::Hazard && hit(b.x - ix, b.y - iy, b.x + ix, b.y + iy, o, MARGIN)) return false;
		}
	}
	return true;
}

// ---------------------------------------------------------------- controllers

static bool cubeWant(Body const& b, std::vector<Obj> const& w, float reach) {
	if (simulate(b, false, false, w, reach)) return false; // staying idle survives
	return simulate(b, true, true, w, reach);              // pressing now survives
}

// Ship / UFO / swing / wave: steer to the middle of the nearest free vertical gap.
static float gapTarget(Body const& b, std::vector<Obj> const& w) {
	float lo = phys::GROUND_Y + b.hh + 4.f;
	float hi = phys::CEIL_Y - b.hh - 4.f;
	float xEnd = b.x + b.speed * 0.45f;

	struct Iv { float a, b; };
	std::vector<Iv> bad;
	for (auto const& o : w) {
		if (o.kind != Kind::Solid && o.kind != Kind::Hazard) continue;
		if (o.x1 < b.x - b.hw || o.x0 > xEnd) continue;
		bad.push_back({o.y0 - b.hh - 6.f, o.y1 + b.hh + 6.f});
	}
	std::sort(bad.begin(), bad.end(), [](Iv const& p, Iv const& q) { return p.a < q.a; });

	std::vector<Iv> freeIv;
	float cur = lo;
	for (auto const& iv : bad) {
		if (iv.a > cur) freeIv.push_back({cur, std::min(iv.a, hi)});
		cur = std::max(cur, iv.b);
	}
	if (cur < hi) freeIv.push_back({cur, hi});
	if (freeIv.empty()) return b.y;

	Iv best = freeIv[0];
	float bestDist = 1e9f;
	for (auto const& f : freeIv) {
		if (f.b < f.a) continue;
		float d = (b.y < f.a) ? f.a - b.y : (b.y > f.b ? b.y - f.b : 0.f);
		if (d < bestDist) { bestDist = d; best = f; }
	}
	if (best.b - best.a < 120.f) return (best.a + best.b) * 0.5f;
	return std::clamp(b.y, best.a, best.b);
}

static bool gapWant(PlayerObject* p, Body const& b, std::vector<Obj> const& w, double now) {
	float target = gapTarget(b, w);
	float pred = b.y + b.vy * 0.10f;
	float err = (target - pred) * b.g; // >0 means "need to go up" in the player's own frame
	constexpr float dead = 6.f;

	if (p->m_isBird) { // UFO: tap-based
		if (now - s_lastTap > 0.12 && err > dead && b.vy * b.g < 120.f) {
			s_lastTap = now;
			return true;
		}
		return false;
	}
	if (err > dead) return true;
	if (err < -dead) return false;
	return s_lastWant; // hysteresis
}

static bool laneWant(Body const& b, std::vector<Obj> const& w, double now) {
	if (now - s_lastFlip < 0.2) return false;
	float trig = b.speed * 0.16f;
	for (auto const& o : w) {
		if (o.kind != Kind::Hazard) continue;
		float dx = o.x0 - (b.x + b.hw);
		if (dx < 0.f || dx > trig) continue;
		if (o.y1 > b.y - b.hh - 4.f && o.y0 < b.y + b.hh + 4.f) {
			s_lastFlip = now;
			return true;
		}
	}
	return false;
}

static float speedOf(float mult) {
	// GD speed multipliers: 0.7, 0.9, 1.1, 1.3, 1.6 -> units/s
	if (mult < 0.8f) return 251.16f;
	if (mult < 1.0f) return 311.58f;
	if (mult < 1.2f) return 387.42f;
	if (mult < 1.45f) return 468.0f;
	return 576.0f;
}

static bool applyReaction(double now, bool want, float delayMs) {
	if (!s_queue.empty() && now < s_queue.back().first - 0.001) s_queue.clear(); // time went backwards (restart)
	s_queue.emplace_back(now, want);
	double cut = now - delayMs / 1000.0;
	while (s_queue.size() > 1 && s_queue[1].first <= cut) s_queue.pop_front();
	return s_queue.front().first <= cut ? s_queue.front().second : false;
}

// ---------------------------------------------------------------- entry point

void tick(GJBaseGameLayer* l) {
	refreshConfig();

	auto setHeld = [&](bool down) {
		if (down != s_held) {
			l->handleButton(down, 1, true);
			s_held = down;
		}
	};

	if (!s_cfg.on) {
		setHeld(false);
		return;
	}

	PlayerObject* p = l->m_player1;
	if (!p || p->m_isDead || p->m_isPlatformer || p->m_isSideways) {
		setHeld(false);
		return;
	}

	if (s_owner != l || (l->m_objects && l->m_objects->count() != s_objCount)) rebuild(l);

	double now = l->m_gameState.m_levelTime;

	Body b;
	b.x = p->getPositionX();
	b.y = p->getPositionY();
	b.vy = static_cast<float>(p->m_yVelocity) * 60.f;
	b.g = p->m_isUpsideDown ? -1.f : 1.f;
	b.speed = speedOf(p->m_playerSpeed);
	b.hw = b.hh = 15.f * (p->m_vehicleSize > 0.f ? p->m_vehicleSize : 1.f);
	b.ground = p->m_isOnGround;

	float reach = s_cfg.lookahead;
	static std::vector<Obj> window;
	gather(b.x - 60.f, b.x + reach + 60.f, window);

	bool want = false;
	if (p->m_isShip || p->m_isBird || p->m_isSwing || p->m_isDart) {
		want = gapWant(p, b, window, now);
	} else if (p->m_isBall || p->m_isSpider) {
		want = laneWant(b, window, now);
	} else {
		want = cubeWant(b, window, reach); // cube and robot
	}
	s_lastWant = want;

	setHeld(applyReaction(now, want, s_cfg.reactionMs));
}

} // namespace pilot
