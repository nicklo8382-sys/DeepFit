#include "Config.h"
#include "Heads.h"
#include <unordered_set>
#include <mutex>

Config& Config::get()
{
	static Config c;
	return c;
}

// 0.4.4: women's heads have their own set (DeepFit_Female.ini: only the per-head settings; everything else is the
// main file's)
Config& Config::female()
{
	static Config c = [] { Config f; f.isFemale = true; return f; }();
	return c;
}

// the per-head settings: how the mouth, lips, face, throat and sounds look and move - kept apart for women's heads and
// written to preset files; the rest (what is read, keys, other mods) is shared
static const std::unordered_set<std::string>& lookKeys()
{
	static const std::unordered_set<std::string> k{
		"LipMargin", "LipDepthSlab", "LipLaneWidth", "UpperLipMax", "LowerLipMax", "CornerMax", "Stretch", "MinOpen", "MinCorner",
		"LowerLipRound", "MouthExtra", "JawUnits", "LipUnits", "WidenUnits", "MorphMax", "LipCloseMax", "UpperLipDown", "UpperLipClose", "LowerLipClose", "LipSmooth", "CornerCloseMax", "LowerLipInward", "JawFollow", "JawFollowMax", "SmoothSpeed", "UpperJawDown", "LowerLipUp", "LipsAlongShaft",
		"FollowAim", "AimRadius", "AimMaxDeg", "ThroatFrom", "ThroatFull", "ThroatShape", "ThroatBand", "ThroatFill", "ThroatRest",
		"ThroatShapeGain", "ThroatShapeMax", "ThroatWideUnits", "ThroatWideExtra", "ThroatDetail", "ThroatMotion", "ThroatMotionLength",
		"LipRound", "MouthWideGain", "MouthWideMax", "CornerOpen", "ThroatAxisFit", "CheekGain", "CheekMax", "CheekInner",
		"GulpSize", "GulpTime", "GagJaw", "GagSqueeze", "GagTime", "ShapeRidge", "ShapeKnob", "TeethOpen", "TeethMove", "TeethHide", "TeethDown",
		"TeethBack", "FaceReact", "FaceEyes", "FaceBrows", "FaceGag", "LipDrag", "LipDragSpeed", "RecoverHold", "RecoverOpen",
		"RecoverGulp", "AutoGagGulp", "AutoDeep", "AutoGagSpeed", "AutoGagCooldown", "AutoGulpHold",
		"SoundGulp", "SoundGag", "SoundGasp", "SoundOwnOnly", "ExtraPush", "ExtraDepth", "ExtraChance", "ExtraHold", "ExtraSwell", "ShapeFlare", "ShapeKnotOpen" };
	return k;
}
bool Config::isLookKey(const std::string& k) { return lookKeys().contains(k); }

static std::string trim(const std::string& s)
{
	auto a = s.find_first_not_of(" \t\r\n");
	auto b = s.find_last_not_of(" \t\r\n");
	return a == std::string::npos ? std::string() : s.substr(a, b - a + 1);
}

static const char* kMainIni = "Data/SKSE/Plugins/DeepFit.ini";
static const char* kFemaleIni = "Data/SKSE/Plugins/DeepFit_Female.ini";

// 0.4.4: one file (the main ini, the women's ini or a preset) read into this set; lookOnly = only the per-head settings
bool Config::loadFile(const std::string& path, bool lookOnly)
{
	std::ifstream f(path);
	if (!f)
		return false;
	std::string line;
	while (std::getline(f, line)) {
		auto hash = line.find_first_of(";#");
		if (hash != std::string::npos)
			line = line.substr(0, hash);
		auto eq = line.find('=');
		if (eq == std::string::npos)
			continue;
		std::string k = trim(line.substr(0, eq));
		std::string v = trim(line.substr(eq + 1));
		if (k.empty() || v.empty())
			continue;
		if (lookOnly && !isLookKey(k))
			continue;
		applyKey(k, v);
	}
	return true;
}

void Config::applyKey(const std::string& k, const std::string& v)
{
	{
		auto fl = [&](float& dst) { try { dst = std::stof(v); } catch (...) {} };
		auto bl = [&](bool& dst) { dst = (v == "1" || v == "true" || v == "True" || v == "yes"); };

		if (k == "Enabled") bl(enabled);
		else if (k == "DebugLog") bl(debugLog);
		else if (k == "MaxPartnerDistance") fl(maxPartnerDistance);
		else if (k == "SmoothSpeed") fl(smoothSpeed);
		else if (k == "CacheRefreshSeconds") fl(cacheRefreshSeconds);
		else if (k == "LipMargin") fl(lipMargin);
		else if (k == "LipDepthSlab") fl(lipDepthSlab);
		else if (k == "LipLaneWidth") fl(lipLaneWidth);
		else if (k == "UpperLipMax") fl(upperLipMax);
		else if (k == "LowerLipMax") fl(lowerLipMax);
		else if (k == "CornerMax") fl(cornerMax);
		else if (k == "Stretch") fl(stretch);
		else if (k == "MinOpen") fl(minOpen);
		else if (k == "MinCorner") fl(minCorner);
		else if (k == "MouthMorphs") bl(mouthMorphs);
		else if (k == "MegaMouth") bl(megaMouth);
		else if (k == "MegaKey") { try { megaKey = std::stoi(v); } catch (...) {} }
		else if (k == "MegaKeyMods") { try { megaKeyMods = std::stoi(v); } catch (...) {} }
		else if (k == "MouthExtra") fl(mouthExtra);
		else if (k == "JawUnits") fl(jawUnits);
		else if (k == "LipUnits") fl(lipUnits);
		else if (k == "WidenUnits") fl(widenUnits);
		else if (k == "MorphMax") fl(morphMax);
		else if (k == "MorphLerpMs") { try { morphLerpMs = std::stoi(v); } catch (...) {} }
		else if (k == "ThroatMorphs") bl(throatMorphs);
		else if (k == "ThroatMorphMax") fl(throatMorphMax);
		else if (k == "ThroatFrom") fl(throatFrom);
		else if (k == "ThroatFull") fl(throatFull);
		else if (k == "ThroatShape") bl(throatShape);
		else if (k == "MalesOnly") bl(malesOnly);
		else if (k == "ThroatAxisFit") bl(throatAxisFit);
		else if (k == "GuessRigs") bl(guessRigs);
		else if (k == "TeethMove") { try { teethMove = std::clamp(std::stoi(v), 0, 2); } catch (...) {} }
		else if (k == "TeethDown") fl(teethDown);
		else if (k == "TeethBack") fl(teethBack);
		else if (k == "TeethParts") teethParts = v;
		else if (k == "FaceReact") bl(faceReact);
		else if (k == "FaceEyes") fl(faceEyes);
		else if (k == "FaceBrows") fl(faceBrows);
		else if (k == "FaceGag") fl(faceGag);
		else if (k == "LipDrag") fl(lipDrag);
		else if (k == "LipDragSpeed") fl(lipDragSpeed);
		else if (k == "RecoverHold") fl(recoverHold);
		else if (k == "RecoverOpen") fl(recoverOpen);
		else if (k == "RecoverGulp") bl(recoverGulp);
		else if (k == "AutoGagGulp") bl(autoGagGulp);
		else if (k == "AutoDeep") fl(autoDeep);
		else if (k == "AutoGagSpeed") fl(autoGagSpeed);
		else if (k == "AutoGagCooldown") fl(autoGagCooldown);
		else if (k == "AutoGulpHold") fl(autoGulpHold);
		else if (k == "RaceStrength") raceStrength = v;
		else if (k == "AnyMeshAtLips") bl(anyMeshAtLips);
		else if (k == "CheekGain") fl(cheekGain);
		else if (k == "CheekMax") fl(cheekMax);
		else if (k == "CheekInner") fl(cheekInner);
		else if (k == "GulpSize") fl(gulpSize);
		else if (k == "GulpTime") fl(gulpTime);
		else if (k == "GagJaw") fl(gagJaw);
		else if (k == "GagSqueeze") fl(gagSqueeze);
		else if (k == "GagTime") fl(gagTime);
		else if (k == "ShapeRidge") fl(shapeRidge);
		else if (k == "ShapeKnob") fl(shapeKnob);
		else if (k == "LipRound") fl(lipRound);
		else if (k == "MouthWideGain") fl(mouthWideGain);
		else if (k == "MouthWideMax") fl(mouthWideMax);
		else if (k == "CornerOpen") fl(cornerOpen);
		else if (k == "ThroatBand") fl(throatBand);
		else if (k == "ThroatFill") fl(throatFill);
		else if (k == "ThroatRest") fl(throatRest);
		else if (k == "ThroatShapeGain") fl(throatShapeGain);
		else if (k == "ThroatShapeMax") fl(throatShapeMax);
		else if (k == "ThroatWideUnits") fl(throatWideUnits);
		else if (k == "ThroatWideExtra") fl(throatWideExtra);
		else if (k == "ThroatDetail") fl(throatDetail);
		else if (k == "ThroatMotion") fl(throatMotion);
		else if (k == "ThroatMotionLength") fl(throatMotionLen);
		else if (k == "TeethOpen") bl(teethOpen);
		else if (k == "LipCloseMax") fl(lipCloseMax);
		else if (k == "TeethHide") bl(teethHide);
		else if (k == "ReadAfterPPA") bl(readAfterPpa);
		else if (k == "UpperLipClose") fl(upperLipClose);
		else if (k == "LowerLipClose") fl(lowerLipClose);
		else if (k == "LipSmooth") fl(lipSmooth);
		else if (k == "CornerCloseMax") fl(cornerCloseMax);
		else if (k == "LowerLipInward") fl(lowerLipInward);
		else if (k == "JawFollow") fl(jawFollow);
		else if (k == "JawFollowMax") fl(jawFollowMax);
		else if (k == "BeardFollow") bl(beardFollow);
		else if (k == "BeardStrength") fl(beardStrength);
		else if (k == "BeardReach") fl(beardReach);
		else if (k == "BeardShave") bl(beardShave);
		else if (k == "UpperJawDown") fl(upperJawDown);
		else if (k == "UpperLipDown") fl(upperLipDown);
		else if (k == "LowerLipUp") fl(lowerLipUp);
		else if (k == "LowerLipRound") fl(lowerLipRound);
		else if (k == "LipsAlongShaft") bl(lipsAlongShaft);
		else if (k == "FollowAim") bl(followAim);
		else if (k == "AimRadius") fl(aimRadius);
		else if (k == "AimMaxDeg") fl(aimMaxDeg);
		else if (k == "NoseFollow") fl(noseFollow);
		else if (k == "NoseMax") fl(noseMax);
		else if (k == "ThroatEnabled") bl(throatEnabled);
		else if (k == "ThroatSkinOffset") fl(throatSkinOffset);
		else if (k == "ThroatMargin") fl(throatMargin);
		else if (k == "ThroatGain") fl(throatGain);
		else if (k == "ThroatMax") fl(throatMax);
		else if (k == "ThroatSlabZ") fl(throatSlabZ);
		else if (k == "ThroatSlabX") fl(throatSlabX);
		else if (k == "SoundGulp") fl(soundGulp);
		else if (k == "ExtraPush") bl(extraPush);
		else if (k == "DetectShapes") bl(detectShapes);
		else if (k == "ShapeFlare") fl(shapeFlare);
		else if (k == "ShapeKnotOpen") fl(shapeKnotOpen);
		else if (k == "ExtraDepth") fl(extraDepth);
		else if (k == "ExtraChance") fl(extraChance);
		else if (k == "ExtraHold") fl(extraHold);
		else if (k == "ExtraSwell") fl(extraSwell);
		else if (k == "SoundGag") fl(soundGag);
		else if (k == "SoundGasp") fl(soundGasp);
		else if (k == "SoundOwnOnly") bl(soundOwnOnly);
	}
}

void Config::load()
{
	if (isFemale) {
		female().syncFromMain();
		return;
	}
	if (!loadFile(kMainIni, false))
		logs::warn("DeepFit.ini not found, using defaults");
	logs::info("DeepFit.ini loaded (enabled={}, throat={}, debug={})", enabled, throatEnabled, debugLog);
	// women's heads: the main set, then their own per-head settings over it
	auto& f = female();
	f = *this;
	f.isFemale = true;
	if (f.loadFile(kFemaleIni, true))
		logs::info("DeepFit_Female.ini loaded (women's heads)");
	Heads::Load();   // 0.4.6: the head profiles (Heads\\*.ini) over the main / women's sets
}

// the shared settings changed in the main set: the women's set takes them, keeping its own per-head ones
void Config::syncFromMain()
{
	auto& f = female();
	const auto mine = f.keyValues(true);
	f = get();
	f.isFemale = true;
	for (auto& [k, v] : mine)
		f.applyKey(k, v);
	Heads::SyncAll();   // 0.4.6: and every head profile
}

// 0.3.0: write the settings back, changing only the values (comments and order stay as they are); keys the file does
// not have yet are added at the end
std::vector<std::pair<std::string, std::string>> Config::keyValues(bool lookOnly) const
{
	std::vector<std::pair<std::string, std::string>> kv;
	auto f2 = [](float v) { char b[32]; std::snprintf(b, sizeof(b), "%.2f", v); return std::string(b); };
	auto b2 = [](bool v) { return std::string(v ? "1" : "0"); };
	kv = {
		{ "Enabled", b2(enabled) }, { "DebugLog", b2(debugLog) },
		{ "BeardFollow", b2(beardFollow) }, { "BeardStrength", f2(beardStrength) }, { "BeardReach", f2(beardReach) }, { "BeardShave", b2(beardShave) },
		{ "LipMargin", f2(lipMargin) }, { "UpperLipMax", f2(upperLipMax) }, { "LowerLipMax", f2(lowerLipMax) }, { "CornerMax", f2(cornerMax) },
		{ "Stretch", f2(stretch) }, { "MinOpen", f2(minOpen) }, { "MinCorner", f2(minCorner) },
		{ "ThroatEnabled", b2(throatEnabled) }, { "ThroatGain", f2(throatGain) }, { "ThroatMax", f2(throatMax) },
		{ "MouthMorphs", b2(mouthMorphs) }, { "MegaMouth", b2(megaMouth) }, { "MegaKey", std::to_string(megaKey) }, { "MegaKeyMods", std::to_string(megaKeyMods) },
		{ "MouthExtra", f2(mouthExtra) }, { "JawUnits", f2(jawUnits) }, { "LipUnits", f2(lipUnits) }, { "WidenUnits", f2(widenUnits) },
		{ "MorphMax", f2(morphMax) }, { "MorphLerpMs", std::to_string(morphLerpMs) }, { "ThroatMorphs", b2(throatMorphs) },
		{ "ThroatMorphMax", f2(throatMorphMax) }, { "ThroatFrom", f2(throatFrom) }, { "ThroatFull", f2(throatFull) },
		{ "MalesOnly", b2(malesOnly) }, { "ThroatAxisFit", b2(throatAxisFit) },
		{ "GuessRigs", b2(guessRigs) }, { "AnyMeshAtLips", b2(anyMeshAtLips) },
		{ "TeethMove", std::to_string(teethMove) }, { "TeethHide", b2(teethHide) }, { "TeethDown", f2(teethDown) }, { "TeethBack", f2(teethBack) }, { "TeethParts", teethParts },
		{ "FaceReact", b2(faceReact) }, { "FaceEyes", f2(faceEyes) }, { "FaceBrows", f2(faceBrows) }, { "FaceGag", f2(faceGag) },
		{ "LipDrag", f2(lipDrag) }, { "LipDragSpeed", f2(lipDragSpeed) },
		{ "RecoverHold", f2(recoverHold) }, { "RecoverOpen", f2(recoverOpen) }, { "RecoverGulp", b2(recoverGulp) },
		{ "AutoGagGulp", b2(autoGagGulp) }, { "AutoDeep", f2(autoDeep) }, { "AutoGagSpeed", f2(autoGagSpeed) },
		{ "AutoGagCooldown", f2(autoGagCooldown) }, { "AutoGulpHold", f2(autoGulpHold) }, { "RaceStrength", raceStrength },
		{ "CheekGain", f2(cheekGain) }, { "CheekMax", f2(cheekMax) }, { "CheekInner", f2(cheekInner) },
		{ "GulpSize", f2(gulpSize) }, { "GulpTime", f2(gulpTime) }, { "GagJaw", f2(gagJaw) }, { "GagSqueeze", f2(gagSqueeze) },
		{ "GagTime", f2(gagTime) }, { "ShapeRidge", f2(shapeRidge) }, { "ShapeKnob", f2(shapeKnob) }, { "LipRound", f2(lipRound) }, { "MouthWideGain", f2(mouthWideGain) },
		{ "MouthWideMax", f2(mouthWideMax) }, { "CornerOpen", f2(cornerOpen) }, { "ThroatShape", b2(throatShape) }, { "ThroatBand", f2(throatBand) }, { "ThroatFill", f2(throatFill) }, { "ThroatRest", f2(throatRest) },
		{ "ThroatShapeGain", f2(throatShapeGain) }, { "ThroatShapeMax", f2(throatShapeMax) }, { "ThroatWideUnits", f2(throatWideUnits) },
		{ "ThroatWideExtra", f2(throatWideExtra) }, { "ThroatDetail", f2(throatDetail) },
		{ "ThroatMotion", f2(throatMotion) }, { "ThroatMotionLength", f2(throatMotionLen) },
		{ "TeethOpen", b2(teethOpen) }, { "LipCloseMax", f2(lipCloseMax) }, { "UpperLipDown", f2(upperLipDown) }, { "UpperLipClose", f2(upperLipClose) }, { "LowerLipClose", f2(lowerLipClose) }, { "LipSmooth", f2(lipSmooth) }, { "CornerCloseMax", f2(cornerCloseMax) }, { "LowerLipInward", f2(lowerLipInward) }, { "JawFollow", f2(jawFollow) }, { "JawFollowMax", f2(jawFollowMax) }, { "SmoothSpeed", f2(smoothSpeed) }, { "UpperJawDown", f2(upperJawDown) },
		{ "LowerLipUp", f2(lowerLipUp) }, { "LowerLipRound", f2(lowerLipRound) }, { "LipsAlongShaft", b2(lipsAlongShaft) },
		{ "ReadAfterPPA", b2(readAfterPpa) }, { "FollowAim", b2(followAim) }, { "AimRadius", f2(aimRadius) }, { "AimMaxDeg", f2(aimMaxDeg) },
		{ "DetectShapes", b2(detectShapes) }, { "ShapeFlare", f2(shapeFlare) }, { "ShapeKnotOpen", f2(shapeKnotOpen) },
		{ "ExtraPush", b2(extraPush) }, { "ExtraDepth", f2(extraDepth) }, { "ExtraChance", f2(extraChance) }, { "ExtraHold", f2(extraHold) }, { "ExtraSwell", f2(extraSwell) },
		{ "SoundGulp", f2(soundGulp) }, { "SoundGag", f2(soundGag) }, { "SoundGasp", f2(soundGasp) },
		{ "SoundOwnOnly", b2(soundOwnOnly) },
	};
	if (lookOnly) {
		std::vector<std::pair<std::string, std::string>> out;
		for (auto& e : kv)
			if (isLookKey(e.first))
				out.push_back(e);
		return out;
	}
	return kv;
}

void Config::save()
{
	if (!ownFile.empty())   // 0.4.6: a head profile: only its per-head settings, into its own file
		saveTo(ownFile, true);
	else if (isFemale)
		saveTo(kFemaleIni, true);
	else {
		saveTo(kMainIni, false);
		syncFromMain();
	}
}

// writes this set into a file: the values of the keys it has are changed (comments and order kept), the rest added
bool Config::saveTo(const std::string& path, bool lookOnly) const
{
	const auto kv = keyValues(lookOnly);
	std::vector<std::string> lines;
	{
		std::ifstream in(path);
		std::string   line;
		while (std::getline(in, line)) {
			if (!line.empty() && line.back() == '\r')
				line.pop_back();
			lines.push_back(line);
		}
	}
	std::vector<bool> done(kv.size(), false);
	for (auto& line : lines) {
		const auto eq = line.find('=');
		const auto semi = line.find_first_of(";#");
		if (eq == std::string::npos || (semi != std::string::npos && semi < eq))
			continue;
		const std::string k = trim(line.substr(0, eq));
		for (std::size_t i = 0; i < kv.size(); ++i) {
			if (done[i] || k != kv[i].first)
				continue;
			const std::string comment = semi != std::string::npos ? line.substr(semi) : std::string();
			std::string out = kv[i].first + " = " + kv[i].second;
			if (!comment.empty())
				out += std::string(out.size() < 29 ? 29 - out.size() : 1, ' ') + comment;
			line = out;
			done[i] = true;
		}
	}
	for (std::size_t i = 0; i < kv.size(); ++i)
		if (!done[i])
			lines.push_back(kv[i].first + " = " + kv[i].second);
	std::ofstream out(path, std::ios::binary | std::ios::trunc);
	if (!out)
		return false;
	for (auto& line : lines)
		out << line << "\r\n";
	return true;
}

// 0.4.0: "WerewolfBeastRace:0.80, OtherRace:1.20" (race editor IDs, any case)
static std::string lower(std::string s)
{
	for (auto& c : s)
		c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	return s;
}

static std::vector<std::pair<std::string, float>> parseRaces(const std::string& s)
{
	std::vector<std::pair<std::string, float>> out;
	std::size_t i = 0;
	while (i < s.size()) {
		std::size_t j = s.find(',', i);
		if (j == std::string::npos) j = s.size();
		const std::string item = trim(s.substr(i, j - i));
		const auto c = item.rfind(':');
		if (c != std::string::npos) {
			try { out.emplace_back(trim(item.substr(0, c)), std::stof(item.substr(c + 1))); } catch (...) {}
		}
		i = j + 1;
	}
	return out;
}

static std::mutex g_raceLock;   // the menu writes it while the game reads it

float Config::raceStrengthFor(const std::string& a_race) const
{
	std::lock_guard lk(g_raceLock);
	if (raceStrength.empty() || a_race.empty())
		return 1.0f;
	const std::string r = lower(a_race);
	for (auto& [n, k] : parseRaces(raceStrength))
		if (lower(n) == r)
			return std::clamp(k, 0.0f, 3.0f);
	return 1.0f;
}

void Config::clearRaces()
{
	std::lock_guard lk(g_raceLock);
	raceStrength.clear();
}

void Config::setRaceStrength(const std::string& a_race, float a_k)
{
	std::lock_guard lk(g_raceLock);
	auto list = parseRaces(raceStrength);
	bool found = false;
	for (auto it = list.begin(); it != list.end();) {
		if (lower(it->first) == lower(a_race)) {
			found = true;
			if (std::fabs(a_k - 1.0f) < 0.005f) { it = list.erase(it); continue; }
			it->second = a_k;
		}
		++it;
	}
	if (!found && std::fabs(a_k - 1.0f) >= 0.005f)
		list.emplace_back(a_race, a_k);
	std::string s;
	for (auto& [n, k] : list) {
		char b[96];
		std::snprintf(b, sizeof(b), "%s%s:%.2f", s.empty() ? "" : ", ", n.c_str(), k);
		s += b;
	}
	raceStrength = s;
}

void Config::resetDefaults()
{
	const int  key = megaKey, mods = megaKeyMods;
	const bool dbg = debugLog;
	const std::string races = raceStrength;
	const bool        fem = isFemale;
	if (fem || !ownFile.empty()) {   // 0.4.4: the women's set (0.4.6: or a head profile's): the per-head settings back to as installed, the shared ones kept
		Config d{};
		for (auto& [k, v] : d.keyValues(true))
			applyKey(k, v);
		save();
		return;
	}
	*this = Config{};
	raceStrength = races;
	megaKey = key;
	megaKeyMods = mods;
	debugLog = dbg;
	save();
}
