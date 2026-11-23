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
	float upperLipMax = 1.5f;      // max push of the upper lip bones
	float lowerLipMax = 2.2f;      // max push of the lower lip bones
	float cornerMax = 1.2f;        // max sideways push of the corner bones
	float stretch = 0.0f;          // 0.1.4: lips pushed this much further than the shaft's surface (stretched round it)
	float minOpen = 1.2f;          // 0.1.4: with a shaft in his mouth each lip opens at least this far
	float minCorner = 0.8f;        // 0.1.4: ... and each corner spreads at least this far
	// 0.2.0: the mouth itself (SMP Head's MFEE morphs PPA_JawWide / PPA_LipStretch / PPA_LipWiden / PPA_TH1-3) -
	// DeepFit opens it to the shaft's thickness at the lips (closes it again round a slimmer one), or as wide as it goes
	bool  mouthMorphs = true;      // DeepFit drives the mouth morphs (turn off to leave them to PPA or another mod)
	bool  megaMouth = false;       // as wide as the morphs go, whatever the shaft
	int   megaKey = 0;             // 0.3.0: a key that switches Mega mouth on / off in game (DirectInput scan code, 0 = none)
	int   megaKeyMods = 0;         // 0.3.1: with Shift (1) / Ctrl (2) / Alt (4) held
	float mouthExtra = 3.0f;       // opened this much more than the shaft is thick
	float jawUnits = 10.0f;         // how far PPA_JawWide at 100 opens the jaw (units)
	float lipUnits = 8.0f;         // how far PPA_LipStretch at 100 rounds the lips out
	float widenUnits = 5.0f;       // how far PPA_LipWiden at 100 spreads each corner
	float morphMax = 300.0f;       // the most any of them is set to (MFEE's own Max caps it too)
	int   morphLerpMs = 120;       // MFEE blend time for each change
	bool  throatMorphs = false;    // PPA_TH1-3 by how deep the tip is (0.3.5: off by default, DF_Throat1-5 instead)
	float throatMorphMax = 120.0f; // 0.3.4: the throat morphs' own most (they share one smooth ridge; MorphMax still caps it)
	float throatFrom = 5.0f;       // 0.3.4: tip this deep (units past the lips) = the throat starts to swell
	float throatFull = 14.0f;      // 0.3.4: this deep = the whole ridge is out
	float lipRound = 0.6f;         // 0.3.8: how much of the lip-rounding morph is used (the rest of the opening goes to the sides)
	float mouthWideGain = 1.0f;    // 0.3.8: DF_MouthWide - units each corner goes out per unit needed
	float mouthWideMax = 1.8f;     // 0.3.8: the most each corner goes out (units)
	float cornerOpen = 0.12f;      // 0.3.8: corners also go out by this share of how far the mouth is open
	bool  guessRigs = true;        // 0.4.0: a skeleton DeepFit does not know: its schlong bones guessed from their names
	bool  anyMeshAtLips = false;   // 0.4.0: still nothing: any of his mesh that reaches the lips counts (heavier)
	bool  throatAxisFit = true;    // 0.4.0: measure the shaft along its own line (angled / bent shafts)
	float cheekGain = 1.0f;        // 0.4.0: a shaft pushed to one side bulges that cheek (units per unit past CheekInner)
	float cheekMax = 1.2f;
	float cheekInner = 1.6f;       // 0.4.0: room inside his mouth to each side before the cheek is pushed
	float gulpSize = 1.2f;         // 0.4.0: a swallow running down his throat (units; 0 = off) - Anatomica's gulps
	float gulpTime = 0.7f;         // seconds from chin to neck
	float gagJaw = 25.0f;          // 0.4.0: a gag drops his jaw this much more (morph), ...
	float gagSqueeze = 0.4f;       // ... and tightens his throat by this share
	float gagTime = 0.9f;
	float shapeRidge = 0.6f;       // 0.4.0: a ridged schlong's ridge down his throat (units)
	float shapeKnob = 0.7f;        // 0.4.0: a segmented schlong's knobs (units)
	bool  malesOnly = true;        // 0.3.7: only male mouths are fitted (women keep SMP Head's own physics / PPA)
	bool  throatShape = true;      // 0.3.5: DF_ThroatFill1-10 / DF_Throat1-10 follow the schlong's own shape (thickness along it)
	float throatBand = 0.6f;       // 0.3.5: units of shaft per throat band (10 bands, from ThroatFrom deep)
	float throatFill = 1.0f;       // 0.3.5: how far out onto the chin-to-seam line his throat fills where the shaft is (0-1)
	float throatRest = 0.6f;       // 0.3.5: an empty throat takes this much half thickness without bulging
	float throatShapeGain = 1.6f;  // 0.3.5: units of bulge per unit of shaft thickness past ThroatRest
	float throatShapeMax = 3.0f;   // 0.3.5: the most a band bulges (units)
	float throatWideUnits = 1.4f;  // 0.3.5: a shaft this much wider than tall (half width) spreads the bulge fully sideways
	float throatWideExtra = 0.3f;  // 0.3.5: every bulge this much wider anyway (0 = as wide as the shaft, 2 = most of his neck)
	float throatMotion = 0.4f;     // 0.3.6: a ripple that slides along his throat with every stroke (units; 0 = off)
	float throatMotionLen = 2.0f;  // 0.3.6: its length along the shaft (units)
	float throatDetail = 1.6f;     // 0.3.5: a flare / ring / knot this many times more pronounced than the shaft's own
	bool  teethOpen = true;        // the SMP Head teeth-open modifier (12) while the shaft is in his mouth
	int   teethMove = 1;           // 0.4.0: teeth moved down / back: 0 off, 1 while the mouth is full, 2 the whole scene (0.4.2)
	float teethDown = 0.3f;        // units
	float teethBack = 0.3f;        // units
	std::string teethParts = "teeth, mouth";   // 0.4.0: the face parts that count as teeth (part of their name, comma separated)
	bool  faceReact = true;        // 0.4.0: eyes close / brows lift with depth, clench in a gag
	float faceEyes = 0.5f;
	float faceBrows = 0.4f;
	float faceGag = 0.8f;
	float lipDrag = 0.35f;         // 0.4.0: lips roll out as he pulls back, tuck in as he pushes in (units)
	float lipDragSpeed = 20.0f;    // stroke speed (units a second) for the full drag
	float recoverHold = 1.0f;      // 0.4.0: after he pulls out the mouth stays open this long (s) ...
	float recoverOpen = 0.35f;     // ... this share of how open it was ...
	bool  recoverGulp = true;      // ... then he swallows
	bool  autoGagGulp = true;      // 0.4.0: gags and swallows by DeepFit itself (Anatomica's come first)
	float autoDeep = 4.0f;         // this far past the top of his throat counts as deep (units)
	float autoGagSpeed = 25.0f;    // a deep thrust this fast (units a second) makes him gag
	float autoGagCooldown = 3.0f;  // s
	float autoGulpHold = 1.5f;     // held deep this long (s) and he swallows
	std::string raceStrength;      // 0.4.0: "RaceEditorID:0.80, OtherRace:1.20" - the partner's race scales the mouth and throat
	float raceStrengthFor(const std::string& a_race) const;
	void  setRaceStrength(const std::string& a_race, float a_k);
	void  clearRaces();
	float lipCloseMax = 2.0f;      // lips may close in this far (round a slim shaft once the jaw is open)
	float lowerLipUp = 0.0f;       // 0.4.1: the lower lip pushed up this far while the mouth is full
	bool  lipsAlongShaft = true;   // 0.4.1: the mouth opens to the shaft measured across it (not straight up and down his face)
	bool  followAim = true;        // 0.4.3: a shaft whose line passes his lips is turned onto them (as PPA aims it)
	float aimRadius = 10.0f;       // ... when it passes within this many units of them
	float aimMaxDeg = 45.0f;       // ... turned at most this far
	float upperLipDown = 0.6f;     // 0.3.3: the upper lip pulled down this far while the mouth is full (the lip morph peaks it up to the nose)
	float noseFollow = 0.5f;       // nose base rises by this fraction of the upper-lip-center push
	float noseMax = 0.5f;

	// throat
	bool  throatEnabled = false;
	float throatSkinOffset = 1.5f;  // distance from each throat bone to the skin surface (along head Y)
	float throatMargin = 0.2f;
	float throatGain = 2.5f;        // throat weights peak ~0.37, so bone push is scaled up
	float throatMax = 0.0f;         // max forward push of a throat bone
	float throatSlabZ = 1.2f;       // +- height band around the throat bone
	float throatSlabX = 2.0f;       // +- sideways band around the throat bone

	// 0.4.4: his schlong's shape found by DeepFit itself (without Anatomica - with it, Anatomica says)
	bool  detectShapes = true;
	float shapeFlare = 0.8f;       // a flared head swells this much more just behind the tip (units)
	float shapeKnotOpen = 1.5f;    // a knot all the way in opens his mouth this much more (units)

	// 0.4.4: an extra push - now and then, at the deepest point of a stroke, the bulge goes further down his throat
	bool  extraPush = true;
	float extraDepth = 4.0f;       // units deeper
	float extraChance = 0.25f;     // the chance each stroke (1 = every stroke)
	float extraHold = 0.8f;        // seconds it stays there

	// 0.4.4: sounds (Data\Sound\Fx\DeepFit\<gulp / gag / gasp>; a swallow falls back to Anatomica's gulps)
	float soundGulp = 0.8f;        // volume of a swallow (0 = off)
	float soundGag = 0.8f;         // a gag
	float soundGasp = 0.6f;        // a breath after he pulls out
	bool  soundOwnOnly = true;     // only DeepFit's own swallows and gags (Anatomica plays its own)

	bool           isFemale = false;   // 0.4.4: the women's set
	static Config& get();               // the main (men's) set, and every shared setting
	static Config& female();            // 0.4.4: women's heads built for DeepFit
	static Config& of(bool a_female) { return a_female ? female() : get(); }
	static bool    isLookKey(const std::string& k);
	void           load();
	void           save();            // 0.3.0: the in-game menu writes its changes back to DeepFit.ini (keeps the comments)
	void           resetDefaults();   // 0.3.1: every setting back to as installed (the key and the debug log are kept)
	bool           loadFile(const std::string& path, bool lookOnly);   // 0.4.4: a preset file too
	bool           saveTo(const std::string& path, bool lookOnly) const;
	void           applyKey(const std::string& k, const std::string& v);
	std::vector<std::pair<std::string, std::string>> keyValues(bool lookOnly) const;
	static void    syncFromMain();
};
