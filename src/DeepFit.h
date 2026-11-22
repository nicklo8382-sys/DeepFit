#pragma once

namespace DeepFit
{
	struct Preview   // 0.4.0: the test page - a made-up schlong in the player's mouth, no scene needed
	{
		bool  on = false;
		float depth = 9.0f;     // how far in its tip goes (units past his lips)
		float thick = 1.3f;     // half its height
		float width = 1.3f;     // half its width
		int   shape = 0;        // 0 plain, 1 horse (flare + medial ring), 2 knot, 3 ridged, 4 segmented, 5 a bump (0.4.1)
		bool  strokes = true;   // goes in and out
		float speed = 0.6f;     // 0.4.1: strokes a second
		float strokeLen = 0.22f;   // 0.4.1: how far it comes out each stroke (share of the depth)
		float bumpAt = 0.5f;    // 0.4.1: the bump: where along it (0 base - 1 tip) ...
		float bumpSize = 0.5f;  // ... and how much thicker there
		float up = 0.0f, side = 0.0f;       // 0.4.1: where it enters: up / to his right of the middle of his lips (units)
		float pitch = 0.0f, yaw = 0.0f;     // 0.4.1: its angle: tip up / tip to his right (degrees)
		float tilt = 0.0f;      // 0.4.1: his head tilted back (degrees) - his throat is easier to see
		int   size = -1;        // 0.4.0: -1 the sliders, 0-4 TNG's XS / S / M / L / XL, 5 the player's own TNG size (a plain shape)
		bool  show = true;      // 0.4.0: a plain schlong shown in his mouth
	};
	struct TngSizes   // 0.4.0: TNG's size scales (its MCM's), and the player's own size category
	{
		float scale[5] = { 0.8f, 0.9f, 1.0f, 1.2f, 1.4f };
		int   mine = -1;        // -1 not known (TNG not there, or no answer yet)
		bool  fromTng = false;
	};
	TngSizes GetTngSizes();
	void     AskTng();          // asks TNG again (on the game's thread, next frame)
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
