#pragma once

struct Config
{
	// general
	bool  enabled = true;
	bool  debugLog = false;
	float maxPartnerDistance = 150.0f;  // units between receiver head and partner pelvis before we bother
	float smoothSpeed = 25.0f;          // 1/s, how fast bones chase their target
	float cacheRefreshSeconds = 2.0f;   // how often shaft vertex caches are rebuilt

	// lips
	float lipMargin = 0.08f;       // gap kept between lip surface and shaft surface
	float lipDepthSlab = 0.8f;     // +- along the mouth axis: shaft verts that count for a lip bone
	float lipLaneWidth = 0.6f;     // +- across the lip: shaft verts that count for a lip bone
	float upperLipMax = 1.6f;      // max push of the upper lip bones
	float lowerLipMax = 2.2f;      // max push of the lower lip bones
	float cornerMax = 1.2f;        // max sideways push of the corner bones
	float noseFollow = 0.5f;       // nose base rises by this fraction of the upper-lip-center push
	float noseMax = 0.5f;

	// throat
	bool  throatEnabled = true;
	float throatSkinOffset = 1.5f;  // distance from each throat bone to the skin surface (along head Y)
	float throatMargin = 0.2f;
	float throatGain = 2.5f;        // throat weights peak ~0.37, so bone push is scaled up
	float throatMax = 4.0f;         // max forward push of a throat bone
	float throatSlabZ = 1.2f;       // +- height band around the throat bone
	float throatSlabX = 2.0f;       // +- sideways band around the throat bone

	static Config& get();
	void           load();
};
