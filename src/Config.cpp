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
