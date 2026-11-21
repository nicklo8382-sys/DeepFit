#include "Config.h"

Config& Config::get()
{
	static Config c;
	return c;
}

static std::string trim(const std::string& s)
{
	auto a = s.find_first_not_of(" \t\r\n");
	auto b = s.find_last_not_of(" \t\r\n");
	return a == std::string::npos ? std::string() : s.substr(a, b - a + 1);
}

void Config::load()
{
	std::ifstream f("Data/SKSE/Plugins/DeepFit.ini");
	if (!f) {
		logs::warn("DeepFit.ini not found, using defaults");
		return;
	}
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
		else if (k == "MouthExtra") fl(mouthExtra);
		else if (k == "JawUnits") fl(jawUnits);
		else if (k == "LipUnits") fl(lipUnits);
		else if (k == "WidenUnits") fl(widenUnits);
		else if (k == "MorphMax") fl(morphMax);
		else if (k == "MorphLerpMs") { try { morphLerpMs = std::stoi(v); } catch (...) {} }
		else if (k == "ThroatMorphs") bl(throatMorphs);
		else if (k == "TeethOpen") bl(teethOpen);
		else if (k == "LipCloseMax") fl(lipCloseMax);
		else if (k == "NoseFollow") fl(noseFollow);
		else if (k == "NoseMax") fl(noseMax);
		else if (k == "ThroatEnabled") bl(throatEnabled);
		else if (k == "ThroatSkinOffset") fl(throatSkinOffset);
		else if (k == "ThroatMargin") fl(throatMargin);
		else if (k == "ThroatGain") fl(throatGain);
		else if (k == "ThroatMax") fl(throatMax);
		else if (k == "ThroatSlabZ") fl(throatSlabZ);
		else if (k == "ThroatSlabX") fl(throatSlabX);
	}
	logs::info("DeepFit.ini loaded (enabled={}, throat={}, debug={})", enabled, throatEnabled, debugLog);
}

// 0.3.0: write the settings back, changing only the values (comments and order stay as they are); keys the file does
// not have yet are added at the end
void Config::save()
{
	const char* path = "Data/SKSE/Plugins/DeepFit.ini";
	std::vector<std::pair<std::string, std::string>> kv;
	auto f2 = [](float v) { char b[32]; std::snprintf(b, sizeof(b), "%.2f", v); return std::string(b); };
	auto b2 = [](bool v) { return std::string(v ? "1" : "0"); };
	kv = {
		{ "Enabled", b2(enabled) }, { "DebugLog", b2(debugLog) },
		{ "LipMargin", f2(lipMargin) }, { "UpperLipMax", f2(upperLipMax) }, { "LowerLipMax", f2(lowerLipMax) }, { "CornerMax", f2(cornerMax) },
		{ "Stretch", f2(stretch) }, { "MinOpen", f2(minOpen) }, { "MinCorner", f2(minCorner) },
		{ "ThroatEnabled", b2(throatEnabled) }, { "ThroatGain", f2(throatGain) }, { "ThroatMax", f2(throatMax) },
		{ "MouthMorphs", b2(mouthMorphs) }, { "MegaMouth", b2(megaMouth) }, { "MegaKey", std::to_string(megaKey) },
		{ "MouthExtra", f2(mouthExtra) }, { "JawUnits", f2(jawUnits) }, { "LipUnits", f2(lipUnits) }, { "WidenUnits", f2(widenUnits) },
		{ "MorphMax", f2(morphMax) }, { "MorphLerpMs", std::to_string(morphLerpMs) }, { "ThroatMorphs", b2(throatMorphs) },
		{ "TeethOpen", b2(teethOpen) }, { "LipCloseMax", f2(lipCloseMax) },
	};
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
	for (auto& line : lines)
		out << line << "\r\n";
}
