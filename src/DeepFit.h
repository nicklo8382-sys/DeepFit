#pragma once

namespace DeepFit
{
	void Install();            // frame driver (at input loaded)
	void OnFrame(float delta);
	bool MegaMouthNow();            // 0.3.0: the setting, or what the key / a mod event switched it to
	bool MouthNow();                // 0.3.0: DeepFit opens the mouth (setting or mod event)
	void SetMegaOverride(int a_value);
	void CaptureKey(bool a_on);      // 0.3.1: the menu waits for a key (with Shift / Ctrl / Alt) for the Mega mouth key
	bool CapturingKey();
	const char* KeyText(int a_code, int a_mods);   // "Shift+Alt+G"   // -1 = as the setting says, 0 off, 1 on // called once per frame from the frame driver
}
