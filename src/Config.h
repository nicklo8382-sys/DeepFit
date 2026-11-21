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
	float stretch = 0.6f;          // 0.1.4: lips pushed this much further than the shaft's surface (stretched round it)
	float minOpen = 1.2f;          // 0.1.4: with a shaft in his mouth each lip opens at least this far
	float minCorner = 0.8f;        // 0.1.4: ... and each corner spreads at least this far
	// 0.2.0: the mouth itself (SMP Head's MFEE morphs PPA_JawWide / PPA_LipStretch / PPA_LipWiden / PPA_TH1-3) -
	// DeepFit opens it to the shaft's thickness at the lips (closes it again round a slimmer one), or as wide as it goes
	bool  mouthMorphs = true;      // DeepFit drives the mouth morphs (turn off to leave them to PPA or another mod)
	bool  megaMouth = false;       // as wide as the morphs go, whatever the shaft
	int   megaKey = 0;             // 0.3.0: a key that switches Mega mouth on / off in game (DirectInput scan code, 0 = none)
	float mouthExtra = 0.3f;       // opened this much more than the shaft is thick
	float jawUnits = 4.5f;         // how far PPA_JawWide at 100 opens the jaw (units)
	float lipUnits = 2.8f;         // how far PPA_LipStretch at 100 rounds the lips out
	float widenUnits = 1.4f;       // how far PPA_LipWiden at 100 spreads each corner
	float morphMax = 100.0f;       // the most any of them is set to (MFEE's own Max caps it too)
	int   morphLerpMs = 120;       // MFEE blend time for each change
	bool  throatMorphs = true;     // PPA_TH1-3 by how deep the tip is
	bool  teethOpen = true;        // the SMP Head teeth-open modifier (12) while the shaft is in his mouth
	float lipCloseMax = 0.8f;      // lips may close in this far (round a slim shaft once the jaw is open)
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
	void           save();   // 0.3.0: the in-game menu writes its changes back to DeepFit.ini (keeps the comments)
};
