#include "Menu.h"
#include "Config.h"
#include "SKSEMenuFramework.h"
#include "DeepFit.h"

// 0.3.0: DeepFit's page in the SKSE Menu Framework. Every change takes effect at once (the next frame) and is written
// back to DeepFit.ini, so it is there after a restart too.
namespace DeepFit::Menu
{
	namespace
	{
		void Help(const char* text)
		{
			ImGuiMCP::SameLine();
			ImGuiMCP::TextDisabled("(?)");
			if (ImGuiMCP::IsItemHovered()) {
				ImGuiMCP::BeginTooltip();
				ImGuiMCP::PushTextWrapPos(420.0f);
				ImGuiMCP::TextUnformatted(text);
				ImGuiMCP::PopTextWrapPos();
				ImGuiMCP::EndTooltip();
			}
		}

		const char* KeyName(int code)
		{
			static char buf[16];
			if (code <= 0)
				return "none";
			std::snprintf(buf, sizeof(buf), "%d", code);
			return buf;
		}

		void __stdcall DrawMouth()
		{
			auto& c = Config::get();
			bool  ch = false;
			ImGuiMCP::TextWrapped("DeepFit moves the mouth of SMP Head (High Poly Head) actors to fit the shaft in it. Changes apply at once and are saved to DeepFit.ini.");
			ch |= ImGuiMCP::Checkbox("Enabled", &c.enabled);

			ImGuiMCP::SeparatorText("Mouth");
			ch |= ImGuiMCP::Checkbox("Mega mouth", &c.megaMouth);
			Help("As wide as the mouth morphs go, whatever the shaft. Off: the mouth opens to the shaft's own thickness where it passes the lips, and closes again round a slimmer part.");
			// 0.3.1: the key set by pressing it (with Shift / Ctrl / Alt if wanted)
			if (DeepFit::CapturingKey()) {
				ImGuiMCP::Text("Mega mouth key: press the key now (Esc to cancel)");
			} else {
				ImGuiMCP::Text("Mega mouth key: %s", DeepFit::KeyText(c.megaKey, c.megaKeyMods));
				ImGuiMCP::SameLine();
				if (ImGuiMCP::Button("Set key"))
					DeepFit::CaptureKey(true);
				ImGuiMCP::SameLine();
				if (ImGuiMCP::Button("No key")) {
					c.megaKey = 0;
					c.megaKeyMods = 0;
					ch = true;
				}
			}
			Help("Switches Mega mouth on and off in game. Press Set key, then the key - hold Shift, Ctrl and / or Alt with it for a combination (e.g. Alt+Shift+G). The key's switch is not saved; the key is.");
			ch |= ImGuiMCP::Checkbox("DeepFit opens the mouth", &c.mouthMorphs);
			Help("DeepFit drives the SMP Head mouth morphs (PPA_JawWide, PPA_LipStretch, PPA_LipWiden, PPA_TH1-3). Off: only the lip bones move, and the morphs are left to PPA or another mod.");
			ch |= ImGuiMCP::SliderFloat("Room round the shaft", &c.mouthExtra, 0.0f, 6.0f, "%.2f");
			Help("The mouth opens this much more than the shaft is thick.");
			ch |= ImGuiMCP::SliderFloat("Jaw (units per 100)", &c.jawUnits, 1.0f, 20.0f, "%.1f");
			Help("How far the jaw morph at 100 opens the jaw. Lower = the jaw opens wider for the same shaft.");
			ch |= ImGuiMCP::SliderFloat("Lips round (units per 100)", &c.lipUnits, 1.0f, 16.0f, "%.1f");
			ch |= ImGuiMCP::SliderFloat("Corners spread (units per 100)", &c.widenUnits, 0.5f, 10.0f, "%.1f");
			ch |= ImGuiMCP::SliderFloat("Most a morph is set to", &c.morphMax, 10.0f, 600.0f, "%.0f");
			Help("MFEE caps the morphs too: its Max in MuFacialExpressionExtended.ini (100 as installed). Raise both for more than the morph's full size.");

			ImGuiMCP::SeparatorText("Lips");
			ch |= ImGuiMCP::SliderFloat("Stretch past the shaft", &c.stretch, -1.0f, 1.0f, "%.2f");
			ch |= ImGuiMCP::SliderFloat("Close in round a slim shaft", &c.lipCloseMax, 0.0f, 4.0f, "%.2f");
			ch |= ImGuiMCP::SliderFloat("Upper lip, at most", &c.upperLipMax, 0.0f, 3.0f, "%.1f");
			ch |= ImGuiMCP::SliderFloat("Upper lip down", &c.upperLipDown, 0.0f, 1.2f, "%.2f");
			Help("While the mouth is full the upper lip is pulled down this far (its middle fully, the sides a little less) - the lip-round morph peaks it up towards the nose.");
			ch |= ImGuiMCP::SliderFloat("Lower lip, at most", &c.lowerLipMax, 0.0f, 4.5f, "%.1f");
			ch |= ImGuiMCP::SliderFloat("Corners, at most", &c.cornerMax, 0.0f, 2.5f, "%.1f");

			ImGuiMCP::SeparatorText("Throat and teeth");
			ch |= ImGuiMCP::Checkbox("Throat bulge (bones)", &c.throatEnabled);
			ch |= ImGuiMCP::Checkbox("Throat bulge (morphs, by depth)", &c.throatMorphs);
			Help("One smooth ridge down the front of his throat (SMP Head's PPA_TH1-3 together), growing from under his chin towards his neck as the tip goes deeper.");
			if (c.throatMorphs) {
				ch |= ImGuiMCP::SliderFloat("Throat ridge, how strong", &c.throatMorphMax, 0.0f, 240.0f, "%.0f");
				ch |= ImGuiMCP::SliderFloat("Starts swelling at", &c.throatFrom, 0.0f, 10.0f, "%.1f units deep");
				ch |= ImGuiMCP::SliderFloat("Fully out at", &c.throatFull, 6.0f, 24.0f, "%.1f units deep");
			}
			ch |= ImGuiMCP::SliderFloat("Throat bulge, at most", &c.throatMax, 0.0f, 8.0f, "%.1f");
			ch |= ImGuiMCP::Checkbox("Teeth part while the mouth is full", &c.teethOpen);

			ImGuiMCP::SeparatorText("Other mods");
			ImGuiMCP::TextWrapped("Other mods can switch Mega mouth with a mod event: SendModEvent(\"DeepFit_MegaMouth\", \"\", 1.0) on, 0.0 off, -1.0 back to the setting here. "
								  "\"DeepFit_Mouth\" the same for DeepFit opening the mouth at all.");
			ch |= ImGuiMCP::Checkbox("Debug log", &c.debugLog);
			ImGuiMCP::Separator();
			if (ImGuiMCP::Button("Reset to defaults"))
				c.resetDefaults();
			Help("Every setting on this page back to as installed (your Mega mouth key and the debug log stay). Each slider's default sits in the middle of its range.");
			if (ch)
				c.save();
		}
	}

	void Register()
	{
		if (!SKSEMenuFramework::IsInstalled()) {
			logs::info("DeepFit: SKSE Menu Framework not found - settings in DeepFit.ini");
			return;
		}
		SKSEMenuFramework::SetSection("DeepFit");
		SKSEMenuFramework::AddSectionItem("Mouth", DrawMouth);
		logs::info("DeepFit: settings page added to the SKSE Menu Framework");
	}
}
