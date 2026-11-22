#pragma once

namespace DeepFit
{
	struct Preview   // 0.4.0: the test page - a made-up schlong in the player's mouth, no scene needed
	{
		bool  on = false;
		float depth = 9.0f;     // how far in its tip goes (units past his lips)
		float thick = 1.3f;     // half its height
		float width = 1.3f;     // half its width
		int   shape = 0;        // 0 plain, 1 horse (flare + medial ring), 2 knot, 3 ridged, 4 segmented
		bool  strokes = true;   // goes in and out
	};
	Preview& GetPreview();
	std::string StatusText(std::string* a_race = nullptr, float* a_ago = nullptr);   // 0.4.0: what DeepFit last found (the menu)
	void Install();            // frame driver (at input loaded)
	void OnFrame(float delta);
	bool MegaMouthNow();            // 0.3.0: the setting, or what the key / a mod event switched it to
	bool MouthNow();                // 0.3.0: DeepFit opens the mouth (setting or mod event)
	void SetMegaOverride(int a_value);
	void CaptureKey(bool a_on);      // 0.3.1: the menu waits for a key (with Shift / Ctrl / Alt) for the Mega mouth key
	bool CapturingKey();
	const char* KeyText(int a_code, int a_mods);   // "Shift+Alt+G"   // -1 = as the setting says, 0 off, 1 on // called once per frame from the frame driver
}
