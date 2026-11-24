#include "Menu.h"
#include "Config.h"
#include "Heads.h"
#include "SKSEMenuFramework.h"
#include "DeepFit.h"
#include <filesystem>
#include <cstring>

// 0.3.0: DeepFit's page in the SKSE Menu Framework. Every change takes effect at once (the next frame) and is written
// back to DeepFit.ini, so it is there after a restart too.
// 0.4.0: a test page, presets, cheeks / gulps / gags / shapes, and translations (Data\SKSE\Plugins\DeepFit\Translations\
// <the game's language>.ini - "en: English text" then "tr: translated text", the same as Anatomica's).
namespace DeepFit::Menu
{
	namespace
	{
		std::unordered_map<std::string, std::string> g_tr;
		bool                                          g_trLoaded = false;

		void LoadTranslations()
		{
			g_trLoaded = true;
			std::string lang;
			if (auto* ini = RE::INISettingCollection::GetSingleton())
				if (auto* set = ini->GetSetting("sLanguage:General"); set && set->GetString())
					lang = set->GetString();
			for (auto& ch : lang)
				ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
			if (lang.empty() || lang == "english")
				return;
			lang[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(lang[0])));
			std::ifstream in("Data/SKSE/Plugins/DeepFit/Translations/" + lang + ".ini", std::ios::binary);
			if (!in)
				return;
			std::string line, en;
			while (std::getline(in, line)) {
				if (!line.empty() && line.back() == '\r')
					line.pop_back();
				if (line.size() >= 3 && line[0] == '\xEF' && line[1] == '\xBB' && line[2] == '\xBF')
					line = line.substr(3);
				if (line.rfind("en: ", 0) == 0)
					en = line.substr(4);
				else if (line.rfind("tr: ", 0) == 0 && !en.empty()) {
					g_tr[en] = line.substr(4);
					en.clear();
				}
			}
			logs::info("DeepFit: {} menu lines translated ({})", g_tr.size(), lang);
		}

		const char* T(const char* en)
		{
			if (!g_trLoaded)
				LoadTranslations();
			if (g_tr.empty())
				return en;
			auto it = g_tr.find(en);
			return it == g_tr.end() ? en : it->second.c_str();
		}

		void Help(const char* text)
		{
			ImGuiMCP::SameLine();
			ImGuiMCP::TextDisabled("(?)");
			if (ImGuiMCP::IsItemHovered()) {
				ImGuiMCP::BeginTooltip();
				ImGuiMCP::PushTextWrapPos(420.0f);
				ImGuiMCP::TextUnformatted(T(text));
				ImGuiMCP::PopTextWrapPos();
				ImGuiMCP::EndTooltip();
			}
		}

		// 0.4.1: MFEE's Max (MuFacialExpressionExtended.ini) - read, and written by the button
		constexpr const char* kMfeeIni = "Data/SKSE/Plugins/MuFacialExpressionExtended.ini";
		bool  g_mfeeChanged = false;
		float g_mfeeMax = -1.0f;
		bool  g_mfeeRead = false;

		float MfeeMax()
		{
			if (g_mfeeRead)
				return g_mfeeMax;
			g_mfeeRead = true;
			std::ifstream in(kMfeeIni);
			std::string   line;
			while (std::getline(in, line)) {
				auto t = line.find_first_not_of(" \t");
				if (t == std::string::npos || line.compare(t, 3, "Max") != 0)
					continue;
				const auto eq = line.find('=');
				if (eq == std::string::npos)
					continue;
				try { g_mfeeMax = std::stof(line.substr(eq + 1)); } catch (...) {}
				break;
			}
			return g_mfeeMax;
		}

		void SetMfeeMax(float v)
		{
			std::vector<std::string> lines;
			{
				std::ifstream in(kMfeeIni, std::ios::binary);
				std::string   line;
				while (std::getline(in, line)) {
					if (!line.empty() && line.back() == '\r')
						line.pop_back();
					lines.push_back(line);
				}
			}
			if (lines.empty())
				return;
			bool done = false;
			for (auto& l : lines) {
				auto t = l.find_first_not_of(" \t");
				if (!done && t != std::string::npos && l.compare(t, 3, "Max") == 0 && l.find('=') != std::string::npos) {
					char b[48];
					std::snprintf(b, sizeof(b), "Max = %.1f", v);
					l = b;
					done = true;
				}
			}
			if (!done)
				return;
			std::ofstream out(kMfeeIni, std::ios::binary | std::ios::trunc);
			for (auto& l : lines)
				out << l << "\r\n";
			g_mfeeChanged = true;
			g_mfeeRead = false;
			logs::info("DeepFit: MFEE's Max set to {:.0f} in its ini (after a restart)", v);
		}

		bool Check(const char* label, bool* v) { return ImGuiMCP::Checkbox(T(label), v); }
		bool Slider(const char* label, float* v, float lo, float hi, const char* fmt) { return ImGuiMCP::SliderFloat(T(label), v, lo, hi, T(fmt)); }
		void Section(const char* t) { ImGuiMCP::SeparatorText(T(t)); }

		// 0.4.0: presets - the throat and mouth sliders set together.
		// 0.4.2: Default is Nick's tuned set; Subtle is a softer version of it, Extreme a stronger one (the teeth placement,
		// what is read and how it is measured are left as they are)
		void Preset(Config& c, int which)
		{
			//                         subtle  default  extreme
			auto v = [which](float s, float d, float e) { return which == 0 ? s : which == 2 ? e : d; };
			c.mouthExtra = v(1.20f, 2.06f, 3.50f);
			c.jawUnits = v(3.60f, 2.70f, 2.00f);
			c.lipUnits = v(9.00f, 7.70f, 6.00f);
			c.widenUnits = v(3.00f, 2.30f, 1.70f);
			c.upperLipMax = v(0.90f, 1.20f, 1.80f);
			c.lowerLipMax = v(1.40f, 1.90f, 2.80f);
			c.cornerMax = v(0.70f, 0.90f, 1.40f);
			c.stretch = v(0.00f, 0.08f, 0.25f);
			c.lipCloseMax = 4.00f;
			c.upperLipDown = v(0.45f, 0.61f, 0.80f);
			c.lowerLipUp = 0.0f;
			c.lipRound = v(0.70f, 0.60f, 0.45f);
			c.mouthWideGain = v(0.80f, 1.00f, 1.40f);
			c.mouthWideMax = v(0.60f, 0.80f, 1.30f);
			c.cornerOpen = v(0.14f, 0.20f, 0.30f);
			c.cheekGain = v(0.60f, 1.00f, 1.60f);
			c.cheekMax = v(1.00f, 1.60f, 2.40f);
			c.cheekInner = v(1.80f, 1.50f, 1.20f);
			c.throatFrom = 6.10f;
			c.throatBand = 1.86f;
			c.throatFill = v(0.80f, 1.00f, 1.00f);
			c.throatRest = v(2.40f, 2.00f, 1.40f);
			c.throatShapeGain = v(1.10f, 1.60f, 2.40f);
			c.throatShapeMax = v(2.00f, 3.00f, 4.00f);
			c.throatWideUnits = v(3.00f, 3.00f, 2.20f);
			c.throatWideExtra = v(0.15f, 0.30f, 0.60f);
			c.throatDetail = v(1.20f, 1.60f, 2.20f);
			c.throatMotion = v(0.20f, 0.40f, 0.80f);
			c.throatMotionLen = 6.00f;
			c.gulpSize = v(0.80f, 1.20f, 1.80f);
			c.gagJaw = v(15.0f, 25.0f, 45.0f);
			c.gagSqueeze = v(0.25f, 0.40f, 0.60f);
			c.shapeRidge = v(0.40f, 0.60f, 0.90f);
			c.shapeKnob = v(0.50f, 0.70f, 1.00f);
			c.faceEyes = v(0.35f, 0.53f, 0.80f);
			c.faceBrows = v(0.35f, 0.53f, 0.80f);
			c.faceGag = v(0.35f, 0.53f, 0.85f);
			c.lipDrag = v(0.30f, 0.53f, 0.80f);
			c.lipDragSpeed = 34.0f;
			c.recoverHold = v(0.80f, 1.00f, 1.60f);
			c.recoverOpen = v(0.20f, 0.29f, 0.45f);
		}

		// 0.4.4: custom presets - Data\SKSE\Plugins\DeepFit\Presets\Male (or Female)\<name>.ini, the per-head settings only
		// (made with "Save as preset", shared between players, e.g. for a UBE or 3BA head built for DeepFit)
		std::string PresetDir(bool fem) { return std::string("Data/SKSE/Plugins/DeepFit/Presets/") + (fem ? "Female" : "Male"); }

		std::vector<std::string> PresetFiles(bool fem)
		{
			std::vector<std::string> out;
			std::error_code          ec;
			for (auto it = std::filesystem::directory_iterator(PresetDir(fem), ec); !ec && it != std::filesystem::directory_iterator(); it.increment(ec)) {
				if (!it->is_regular_file(ec))
					continue;
				auto ext = it->path().extension().string();
				for (auto& ch : ext) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
				if (ext == ".ini")
					out.push_back(it->path().stem().string());
			}
			std::sort(out.begin(), out.end());
			return out;
		}

		bool PresetsFor(Config& c, bool fem)
		{
			bool ch = false;
			ImGuiMCP::PushID(fem ? "presets_f" : "presets_m");
			ImGuiMCP::Text("%s", T("Built in:"));
			ImGuiMCP::SameLine();
			if (ImGuiMCP::Button(T("Subtle"))) { Preset(c, 0); ch = true; }
			ImGuiMCP::SameLine();
			if (ImGuiMCP::Button(T("Default"))) { Preset(c, 1); ch = true; }
			ImGuiMCP::SameLine();
			if (ImGuiMCP::Button(T("Extreme"))) { Preset(c, 2); ch = true; }
			Help("Sets the mouth, lips, cheeks, throat, swallow / gag, face and lip-drag sliders together - Default is the tuned set, Subtle softer, Extreme stronger. Fine-tune them below afterwards. Teeth placement and what DeepFit reads are left as they are.");
			static std::vector<std::string> files[2];
			static bool                     looked[2] = { false, false };
			if (!looked[fem]) {
				looked[fem] = true;
				files[fem] = PresetFiles(fem);
			}
			ImGuiMCP::Text("%s", T("Your presets:"));
			if (files[fem].empty()) {
				ImGuiMCP::SameLine();
				ImGuiMCP::TextDisabled("%s", T("none yet"));
			}
			for (auto& n : files[fem]) {
				ImGuiMCP::SameLine();
				if (ImGuiMCP::Button(n.c_str())) {
					if (c.loadFile(PresetDir(fem) + "/" + n + ".ini", true)) {
						ch = true;
						logs::info("DeepFit: preset \"{}\" loaded ({})", n, fem ? "women" : "men");
					}
				}
			}
			ImGuiMCP::SameLine();
			if (ImGuiMCP::SmallButton(T("Refresh")))
				looked[fem] = false;
			static char name[2][64] = { "", "" };
			ImGuiMCP::InputText(T("Name"), name[fem], sizeof(name[fem]));
			ImGuiMCP::SameLine();
			if (ImGuiMCP::Button(T("Save as preset")) && name[fem][0]) {
				std::string n = name[fem];
				for (auto& ch2 : n)
					if (std::strchr("\\/:*?\"<>|", ch2)) ch2 = '_';
				std::error_code ec;
				std::filesystem::create_directories(PresetDir(fem), ec);
				if (c.saveTo(PresetDir(fem) + "/" + n + ".ini", true))
					logs::info("DeepFit: preset \"{}\" saved ({})", n, fem ? "women" : "men");
				looked[fem] = false;
				name[fem][0] = 0;
			}
			Help("Saves every per-head setting (mouth, lips, face, throat, sounds) as a preset file in Data\\SKSE\\Plugins\\DeepFit\\Presets\\Male (or Female). Share the file with a head you built for DeepFit (a UBE or 3BA head, say) - others put it in the same folder and load it here.");
			ImGuiMCP::PopID();
			return ch;
		}

		// each page: the men's settings, then the same for women's heads in a drop-down at the bottom
		void Page(bool (*body)(Config&, bool))
		{
			auto& m = Config::get();
			// 0.4.6: the main (men's / human) set in a drop-down too, like the women's and each head profile's (open at first)
			if (ImGuiMCP::CollapsingHeader(T("Men's heads (human)"), ImGuiMCP::ImGuiTreeNodeFlags_DefaultOpen)) {
				ImGuiMCP::TextDisabled("%s", T("Every man's head without a head profile below; saved to DeepFit.ini."));
				ImGuiMCP::PushID("men");
				if (body(m, false))
					m.save();
				ImGuiMCP::PopID();
			}
			ImGuiMCP::Spacing();
			if (ImGuiMCP::CollapsingHeader(T("Women's heads (built for DeepFit)"))) {
				auto& f = Config::female();
				if (m.malesOnly)
					ImGuiMCP::TextColored(ImGuiMCP::ImVec4{ 1.0f, 0.75f, 0.3f, 1.0f }, "%s", T("Women's heads are off - switch on \"Women's heads too\" on the Info page to use these."));
				ImGuiMCP::TextDisabled("%s", T("Used for women whose head is built for DeepFit; saved to DeepFit_Female.ini."));
				ImGuiMCP::PushID("female");
				if (body(f, true))
					f.save();
				ImGuiMCP::PopID();
			}
			// 0.4.6: every head profile (Heads\*.ini) - its own settings, saved into its file
			auto& heads = Heads::All();
			for (std::size_t i = 0; i < heads.size(); ++i) {
				auto&       h = heads[i];
				std::string label = h.name + (h.sex == 1 ? std::string(T(" (women)")) : h.sex == -1 ? std::string(T(" (men and women)")) : std::string()) + "###head" + std::to_string(i);
				if (!ImGuiMCP::CollapsingHeader(label.c_str()))
					continue;
				std::string who;
				for (auto& r : h.races) who += (who.empty() ? "" : ", ") + r;
				for (auto& m : h.meshes) who += (who.empty() ? "" : ", ") + m;
				ImGuiMCP::TextDisabled("%s %s", T("Used for:"), who.empty() ? T("(nothing set - add Races or HeadMeshes to its file)") : who.c_str());
				ImGuiMCP::TextDisabled("%s %s", T("Saved to"), std::filesystem::path(h.file).filename().string().c_str());
				ImGuiMCP::PushID(static_cast<int>(1000 + i));
				if (body(h.cfg, h.sex == 1))
					h.cfg.save();
				ImGuiMCP::PopID();
			}
		}

		bool LipsBody(Config& c, bool)
		{
			bool ch = false;
			Section("Lips round the shaft");
			ch |= Slider("Stretch past the shaft", &c.stretch, -1.0f, 1.0f, "%.2f");
			ch |= Slider("Close in round a slim shaft", &c.lipCloseMax, 0.0f, 4.0f, "%.2f");
			ch |= Slider("Upper lip, at most", &c.upperLipMax, 0.0f, 3.0f, "%.1f");
			ch |= Slider("Upper lip down", &c.upperLipDown, -2.0f, 3.0f, "%.2f");
			Help("While the mouth is full the upper lip is pulled down this far (its middle fully, the sides a little less) - the lip-round morph peaks it up towards the nose. Below 0 it is lifted instead (a cat's muzzle rising off the shaft).");
			ch |= Slider("Upper lip closes onto the shaft", &c.upperLipClose, 1.0f, 3.0f, "x%.2f");
			ch |= Slider("Upper jaw down (snouts)", &c.upperJawDown, 0.0f, 1.0f, "%.2f");
			Help("A beast's whole upper snout tilts down onto the shaft as the jaw opens (1 = 10 degrees, about 1.8 units at the tip of an Argonian's). Only heads that have DeepFit's DF_UpperJawDown morph (the Argonian High Poly Head) - others ignore it.");
			Help("On a shallow stroke the thinner front of the shaft rides low in his mouth and the upper lip has further to come down. Its bones are only half weighted in the head, so they need to move about twice as far as the lip should: raise this if the upper lip floats above the shaft (2 = about right for SMP Head). Needs \"Close in round a slim shaft\" above 0.");
			ch |= Slider("Lower lip, at most", &c.lowerLipMax, 0.0f, 4.5f, "%.1f");
			ch |= Slider("Lower lip up", &c.lowerLipUp, 0.0f, 1.2f, "%.2f");
			Help("While the mouth is full the lower lip is pushed up this far (its middle fully, the sides a little less), closer round the shaft.");
			ch |= Slider("Round the lower lip", &c.lowerLipRound, 0.0f, 1.5f, "%.2f units");
			Help("As the jaw opens, the sides of the lower lip (between its middle and the corners) push out and down, so the opening is round instead of a diamond pinched in at the sides of the lower lip. 0 = off. Needs the 0.4.4 head morphs (DF_LowerLipRound).");
			Section("Corners");
			ch |= Slider("Corners, at most", &c.cornerMax, 0.0f, 2.5f, "%.1f");
			ch |= Slider("Mouth opens to the sides", &c.mouthWideGain, 0.0f, 2.5f, "x%.2f");
			Help("The corners of his mouth pull out to the sides to the schlong's width (DeepFit's DF_MouthWide morph), so the mouth opens wide round it instead of puckering.");
			ch |= Slider("Corners out, at most", &c.mouthWideMax, 0.0f, 3.0f, "%.1f units");
			ch |= Slider("Corners out with the jaw", &c.cornerOpen, 0.0f, 0.4f, "%.2f");
			ch |= Slider("Lips round forward", &c.lipRound, 0.0f, 1.0f, "%.2f");
			Help("How much the lips push forward into an O. Lower = less of a pucker, more of a wide-open mouth (1 = as before 0.3.8).");
			Section("With each stroke");
			ch |= Slider("Lips drag with the stroke", &c.lipDrag, 0.0f, 1.0f, "%.2f units");
			Help("His lips roll out a little as the shaft pulls back and tuck in as it pushes in. 0 = off.");
			ch |= Slider("Stroke speed for full drag", &c.lipDragSpeed, 5.0f, 60.0f, "%.0f units/s");
			return ch;
		}

		bool MouthBody(Config& c, bool fem)
		{
			bool ch = false;
			if (!fem) {   // shared by both
				Section("Opening");
				ch |= Check("Mega mouth", &c.megaMouth);
				Help("As wide as the mouth morphs go, whatever the shaft. Off: the mouth opens to the shaft's own thickness where it passes the lips, and closes again round a slimmer part.");
				if (DeepFit::CapturingKey()) {
					ImGuiMCP::Text("%s", T("Mega mouth key: press the key now (Esc to cancel)"));
				} else {
					ImGuiMCP::Text("%s %s", T("Mega mouth key:"), DeepFit::KeyText(c.megaKey, c.megaKeyMods));
					ImGuiMCP::SameLine();
					if (ImGuiMCP::Button(T("Set key")))
						DeepFit::CaptureKey(true);
					ImGuiMCP::SameLine();
					if (ImGuiMCP::Button(T("No key"))) {
						c.megaKey = 0;
						c.megaKeyMods = 0;
						ch = true;
					}
				}
				Help("Switches Mega mouth on and off in game. Press Set key, then the key - hold Shift, Ctrl and / or Alt with it for a combination (e.g. Alt+Shift+G). The key's switch is not saved; the key is.");
				ch |= Check("DeepFit opens the mouth", &c.mouthMorphs);
				Help("DeepFit drives the SMP Head mouth morphs (PPA_JawWide, PPA_LipStretch, PPA_LipWiden) and its own. Off: only the lip bones move, and the morphs are left to PPA or another mod.");
			} else
				Section("Opening");
			ch |= Slider("Room round the shaft", &c.mouthExtra, 0.0f, 6.0f, "%.2f");
			Help("The mouth opens this much more than the shaft is thick.");
			ch |= Slider("Jaw (units per 100)", &c.jawUnits, 1.0f, 20.0f, "%.1f");
			Help("How far the jaw morph at 100 opens the jaw. Lower = the jaw opens wider for the same shaft.");
			ch |= Slider("Lips round (units per 100)", &c.lipUnits, 1.0f, 16.0f, "%.1f");
			ch |= Slider("Corners spread (units per 100)", &c.widenUnits, 0.5f, 10.0f, "%.1f");
			ch |= Slider("Most a morph is set to", &c.morphMax, 0.0f, 300.0f, "%.0f");
			Help("MFEE caps the morphs too: its Max in MuFacialExpressionExtended.ini (100 as installed). Raise both for more than the morph's full size.");
			if (!fem) {   // 0.4.1: MFEE's own cap - it reads it only when the game starts (it has no way to change it while playing)
				const float mx = MfeeMax();
				if (mx > 0.0f)
					ImGuiMCP::TextDisabled(T("MFEE's limit (its ini): %.0f"), mx);
				if (mx > 0.0f && mx < 299.5f) {
					ImGuiMCP::SameLine();
					if (ImGuiMCP::Button(T("Raise MFEE's limit to 300")))
						SetMfeeMax(300.0f);
				} else if (mx >= 299.5f) {
					ImGuiMCP::SameLine();
					if (ImGuiMCP::Button(T("Put MFEE's limit back to 100")))
						SetMfeeMax(100.0f);
				}
				if (g_mfeeChanged)
					ImGuiMCP::TextColored(ImGuiMCP::ImVec4{ 1.0f, 0.75f, 0.3f, 1.0f }, "%s", T("MFEE's limit is changed in its ini - restart the game for it to take effect (MFEE only reads it at start)."));
				else
					Help("MFEE has no way to change its limit while the game runs: the button writes it to MFEE's ini, and it takes effect after a restart. It is MFEE's limit for every morph it drives, not only DeepFit's.");
			}
			ch |= Check("Open to the shaft's real thickness", &c.lipsAlongShaft);
			Help("The mouth opens to the shaft measured straight across it. Off: measured straight up and down his face, so a shaft going in at an angle opens it wider (as before 0.4.1).");
			ch |= Check("Follow PPA's aim into his mouth", &c.followAim);
			Help("PPA bends the partner's schlong into his mouth after DeepFit has read where the animation put it. When an animation leaves it a few units off (under his chin), the mouth stayed shut although on screen it was in. On: a shaft whose line passes close by his lips is turned onto them, the way PPA aims it.");
			if (c.followAim) {
				ch |= Slider("Aim when it passes within", &c.aimRadius, 2.0f, 40.0f, "%.1f units of his lips");
				ch |= Slider("Turn it at most", &c.aimMaxDeg, 5.0f, 90.0f, "%.0f degrees");
			}
			Section("Cheeks");
			ch |= Slider("Cheek bulge", &c.cheekGain, 0.0f, 3.0f, "x%.1f");
			Help("A shaft pushed to one side of his mouth bulges that cheek out (SMP Head's cheek bones). 0 = off.");
			ch |= Slider("Cheek, at most", &c.cheekMax, 0.0f, 3.0f, "%.1f units");
			ch |= Slider("Room before the cheek", &c.cheekInner, 0.5f, 3.0f, "%.1f units");
			Section("Teeth");
			ch |= Check("Teeth part while the mouth is full", &c.teethOpen);
			ch |= Check("Hide his teeth during a scene", &c.teethHide);
			Help("His teeth and tongue (the parts named in TeethParts) are hidden while he is in a scene and shown again after. Beast heads have it on: their jaw drops further than their teeth go with it.");
			{
				static const char* kTeeth[] = { "Off", "While the mouth is full", "The whole scene" };
				ImGuiMCP::Text("%s", T("Move the teeth:"));
				for (int i = 0; i < 3; ++i) {
					ImGuiMCP::SameLine();
					if (ImGuiMCP::RadioButton(T(kTeeth[i]), c.teethMove == i)) {
						c.teethMove = i;
						ch = true;
					}
				}
				Help("Moves his teeth a little down and back so they sit clear of the shaft, only in scenes: while his mouth is full, or the whole time a partner's schlong is near him. Outside scenes they are put back exactly where the game has them. If the game rebuilds his face mid-scene, DeepFit moves them again within a second.");
				if (c.teethMove > 0) {
					ch |= Slider("Teeth down", &c.teethDown, -1.0f, 2.0f, "%.2f units");
					ch |= Slider("Teeth back", &c.teethBack, -1.0f, 2.0f, "%.2f units");
				}
			}
			Section("After he pulls out");
			ch |= Slider("Mouth stays open for", &c.recoverHold, 0.0f, 3.0f, "%.1f s");
			Help("After the shaft leaves his mouth it stays a little open for a moment before it closes. 0 = closes straight away.");
			ch |= Slider("How open it stays", &c.recoverOpen, 0.0f, 1.0f, "%.2f");
			ch |= Check("Then he swallows", &c.recoverGulp);
			return ch;
		}

		bool FaceBody(Config& c, bool)
		{
			bool ch = false;
			ch |= Check("His face reacts", &c.faceReact);
			Help("His eyes close and his brows lift the deeper it goes; in a gag his eyes clench and his brows knit. Goes over OStim's own expression, never below it.");
			if (c.faceReact) {
				ch |= Slider("Eyes close with depth", &c.faceEyes, 0.0f, 1.0f, "%.2f");
				ch |= Slider("Brows lift with depth", &c.faceBrows, 0.0f, 1.0f, "%.2f");
				ch |= Slider("Clench in a gag", &c.faceGag, 0.0f, 1.0f, "%.2f");
			}
			return ch;
		}

		bool ThroatBody(Config& c, bool)
		{
			bool ch = false;
			ch |= Check("Throat bulge, by the schlong's shape", &c.throatShape);
			Help("DeepFit's own throat morphs: ten thin bands down the front of his throat, from under his chin to just above the neck seam. Where the shaft is, the throat fills out to a straight line from chin to neck, then swells past it by how thick that part of the schlong is, so a flared head, a medial ring or a knot shows as it goes down.");
			if (c.throatShape) {
				if (ImGuiMCP::Button(T("Throat back to recommended"))) {
					c.throatFrom = 6.1f; c.throatBand = 1.86f; c.throatFill = 1.0f; c.throatRest = 1.0f; c.throatShapeGain = 1.6f;
					c.throatDetail = 1.6f; c.throatShapeMax = 3.0f; c.throatWideExtra = 0.3f; c.throatWideUnits = 3.0f; c.throatMotion = 0.4f;
					ch = true;
				}
				Help("Where and how much: the throat starts 6.1 units past the lips and shows 18.6 units of the shaft, so a flared head, a ring or a knot each lands on its own part of his throat. Very short bands, a low \"Bulge, at most\" or \"Shape stands out\" at 1 make every schlong look the same.");
				Section("Where");
				ImGuiMCP::TextDisabled(T("His throat shows the shaft from %.1f to %.1f units past his lips."), c.throatFrom, c.throatFrom + 10.0f * c.throatBand);
				ch |= Slider("Throat starts at", &c.throatFrom, 0.0f, 12.0f, "%.1f units deep");
				Help("How far past his lips the schlong has to be to reach the top of his throat (under his chin).");
				ch |= Slider("Shaft per band", &c.throatBand, 0.3f, 3.0f, "%.2f units");
				Help("How much of the shaft's length each of the ten bands stands for (smaller = the throat shows a shorter stretch of it, in more detail).");
				ch |= Check("Measure along the shaft", &c.throatAxisFit);
				Help("Measures the shaft along its own line, so one that goes in at an angle or bends reads its real thickness. Off: sliced straight back from his lips.");
				Section("How much");
				ch |= Slider("Fill out to the line", &c.throatFill, 0.0f, 1.0f, "%.2f");
				ch |= Slider("Room in an empty throat", &c.throatRest, 0.0f, 2.0f, "%.1f");
				Help("A schlong thinner than this (half its thickness, in units) passes without a bulge.");
				ch |= Slider("Bulge per thickness", &c.throatShapeGain, 0.2f, 4.0f, "x%.1f");
				ch |= Slider("Shape stands out", &c.throatDetail, 1.0f, 3.0f, "x%.1f");
				Help("A flared head, medial ring or knot this many times more pronounced than the shaft itself (1 = exactly as the schlong is shaped).");
				ch |= Slider("Bulge, at most", &c.throatShapeMax, 0.5f, 3.0f, "%.1f units");
				Help("3 units is as far as the head's throat morphs go (MFEE's limit of 100) - past that every bulge would look the same.");
				ch |= Slider("Wider", &c.throatWideExtra, 0.0f, 1.0f, "%.2f");
				Help("Every bulge spreads further across his neck as soft shoulders round the shaft's own ridge (0 = as wide as the shaft, 1 = most of the front of his neck).");
				ch |= Slider("Spreads fully sideways at", &c.throatWideUnits, 0.3f, 3.0f, "%.1f units wider");
				Help("A schlong wider than it is tall spreads the bulge out to the sides; this much wider (half width over half height) spreads it all the way.");
				ch |= Slider("Moves with each stroke", &c.throatMotion, 0.0f, 1.5f, "%.2f units");
				Help("A soft ripple that slides along his throat as the shaft goes in and out, so it still moves when his throat is as full as it gets.");
				ch |= Slider("Ripple length", &c.throatMotionLen, 0.5f, 6.0f, "%.1f units");
				ch |= Slider("Ridged schlong's ridge", &c.shapeRidge, 0.0f, 2.0f, "%.2f units");
				ch |= Slider("Segmented schlong's knobs", &c.shapeKnob, 0.0f, 2.0f, "%.2f units");
				ch |= Slider("Flared head", &c.shapeFlare, 0.0f, 2.0f, "%.2f units");
				Help("A flared (horse) schlong's head swells this much more just behind the tip, even when his mesh does not read as flared.");
				ch |= Slider("A knot all the way in opens the mouth", &c.shapeKnotOpen, 0.0f, 4.0f, "%.1f units");
				Help("The schlong's shape (ridged, segmented, flared, knotted) comes from Anatomica when it is installed (the same shape its belly bulge uses), otherwise DeepFit finds it itself - see Info, \"Schlong shapes\".");
			}
			Section("Extra push");
			ch |= Check("Now and then it goes deeper", &c.extraPush);
			Help("At the deepest point of a stroke, now and then, the bulge goes further down his throat for a moment - as if pushed in harder - then eases back to where the shaft really is. It reaches at most the bottom of his throat (the neck seam).");
			if (c.extraPush) {
				ch |= Slider("How much deeper", &c.extraDepth, 0.5f, 12.0f, "%.1f units");
				ch |= Slider("Chance each stroke", &c.extraChance, 0.0f, 1.0f, "%.2f");
				Help("0.25 = about one stroke in four; 1 = every stroke.");
				ch |= Slider("Stays there for", &c.extraHold, 0.2f, 3.0f, "%.1f s");
				ch |= Slider("Throat swells meanwhile", &c.extraSwell, 0.0f, 2.0f, "%.2f units");
				Help("While pushed deeper, the throat also swells this much more. Going deeper only shows while the shaft does not already reach the bottom of his throat (\"His throat shows the shaft from ... to ...\" on this page); the swell shows at any depth.");
			}
			Section("Swallow and gag");
			ch |= Slider("Swallow", &c.gulpSize, 0.0f, 3.0f, "%.1f units");
			Help("Each of Anatomica's gulps (when he swallows a finish) runs down his throat as a ripple. 0 = off.");
			ch |= Slider("Swallow time", &c.gulpTime, 0.3f, 2.0f, "%.1f s");
			ch |= Slider("Gag: jaw drops", &c.gagJaw, 0.0f, 80.0f, "%.0f");
			Help("When Anatomica makes him gag, his jaw drops a little further and his throat tightens for a moment.");
			ch |= Slider("Gag: throat tightens", &c.gagSqueeze, 0.0f, 1.0f, "%.2f");
			ch |= Check("Gags and swallows by themselves", &c.autoGagGulp);
			Help("DeepFit makes him gag on a fast, deep thrust and swallow when it is held deep. When Anatomica sends its own gags and gulps, those are used instead.");
			if (c.autoGagGulp) {
				ch |= Slider("Deep is past the throat's top by", &c.autoDeep, 0.0f, 10.0f, "%.1f units");
				ch |= Slider("Gag at a thrust of", &c.autoGagSpeed, 5.0f, 80.0f, "%.0f units/s");
				ch |= Slider("Time between gags", &c.autoGagCooldown, 0.5f, 10.0f, "%.1f s");
				ch |= Slider("Swallow when held deep for", &c.autoGulpHold, 0.5f, 5.0f, "%.1f s");
			}
			return ch;
		}

		bool SoundsBody(Config& c, bool fem)
		{
			bool ch = false;
			ch |= Slider("Swallow", &c.soundGulp, 0.0f, 1.5f, "%.2f volume");
			if (!fem) {
				ImGuiMCP::SameLine();
				ImGuiMCP::TextDisabled(T("(%d files)"), DeepFit::SoundCount("gulp"));
			}
			ch |= Slider("Gag", &c.soundGag, 0.0f, 1.5f, "%.2f volume");
			if (!fem) {
				ImGuiMCP::SameLine();
				ImGuiMCP::TextDisabled(T("(%d files)"), DeepFit::SoundCount("gag"));
			}
			ch |= Slider("Breath after he pulls out", &c.soundGasp, 0.0f, 1.5f, "%.2f volume");
			if (!fem) {
				ImGuiMCP::SameLine();
				ImGuiMCP::TextDisabled(T("(%d files)"), DeepFit::SoundCount("gasp"));
			}
			ch |= Check("Only DeepFit's own swallows and gags", &c.soundOwnOnly);
			Help("Anatomica plays its own gulp sounds when it makes him swallow. On: DeepFit only sounds the swallows and gags it makes itself (held deep, a fast deep thrust, after he pulls out). Off: every one, Anatomica's too.");
			if (!fem) {
				ImGuiMCP::TextWrapped("%s", T("Sound files: WAV or XWM in Data\\Sound\\Fx\\DeepFit\\gulp, \\gag and \\gasp - any number, one picked at random each time. With none for swallows, Anatomica's gulp sounds are used. 0 volume = off."));
				if (ImGuiMCP::Button(T("Look for sound files again")))
					DeepFit::RescanSounds();
			}
			return ch;
		}

		void __stdcall DrawLips() { Page(LipsBody); }
		void __stdcall DrawMouthPage() { Page(MouthBody); }
		void __stdcall DrawFace() { Page(FaceBody); }
		void __stdcall DrawThroat() { Page(ThroatBody); }
		void __stdcall DrawSounds() { Page(SoundsBody); }

		void __stdcall DrawTesting()
		{
			ImGuiMCP::TextWrapped("%s", T("A made-up schlong goes into the player's mouth, so you can tune the mouth and throat here without starting a scene. Not saved - switch it off when done. Needs a head built for DeepFit on the player."));
			auto& pv = DeepFit::GetPreview();
			Check("Show a test schlong in my mouth", &pv.on);
			if (pv.on) {
				Check("Show it", &pv.show);
				Help("A plain schlong is shown in his mouth where the test one is (only its thickness and depth - the shapes below are felt by the mouth and throat, not drawn).");
				Slider("Tip depth", &pv.depth, 0.5f, 30.0f, "%.1f units past the lips");
				{
					const auto ts = DeepFit::GetTngSizes();
					static const char* kSizes[] = { "XS", "S", "M", "L", "XL" };
					ImGuiMCP::Text("%s", T("Size:"));
					ImGuiMCP::SameLine();
					if (ImGuiMCP::RadioButton(T("Sliders"), pv.size < 0))
						pv.size = -1;
					ImGuiMCP::SameLine();
					if (ImGuiMCP::RadioButton(T("Mine (TNG)"), pv.size == 5)) {
						pv.size = 5;
						DeepFit::AskTng();
					}
					for (int i = 0; i < 5; ++i) {
						ImGuiMCP::SameLine();
						if (ImGuiMCP::RadioButton(kSizes[i], pv.size == i))
							pv.size = i;
					}
					Help("A plain schlong the size of one of TNG's size categories, or of the player's own TNG size. Each size uses your TNG size modifier for it - the multiplier set for that size in TNG's MCM (0.8 / 0.9 / 1.0 / 1.2 / 1.4 as TNG installs them); TNG's per-race multipliers are not included. Sliders: the height, width and shape below.");
					ImGuiMCP::TextDisabled("%s", T("Sizes follow your TNG size modifiers (TNG's MCM)."));
					if (pv.size == 5)
						ImGuiMCP::TextDisabled(T("Your TNG size: %s (x%.2f)%s"), ts.mine >= 0 ? kSizes[ts.mine] : "?", ts.scale[ts.mine >= 0 ? ts.mine : 2],
							ts.fromTng ? "" : T(" - TNG did not answer, M used"));
				}
				if (pv.size < 0) {
					Slider("Half its height", &pv.thick, 0.4f, 3.0f, "%.2f");
					Slider("Half its width", &pv.width, 0.4f, 3.5f, "%.2f");
					static const char* kShapes[] = { "Plain", "Horse (flare and medial ring)", "Knot", "Ridged", "Segmented", "Bump" };
					ImGuiMCP::Text("%s", T("Shape:"));
					for (int i = 0; i < 6; ++i) {
						if (i > 0)
							ImGuiMCP::SameLine();
						if (ImGuiMCP::RadioButton(T(kShapes[i]), pv.shape == i))
							pv.shape = i;
					}
					if (pv.shape == 5) {
						Slider("Bump along it", &pv.bumpAt, 0.0f, 1.0f, "%.2f (0 base, 1 tip)");
						Slider("Bump size", &pv.bumpSize, 0.1f, 1.5f, "x%.2f thicker");
					}
				}
				Check("Goes in and out", &pv.strokes);
				if (pv.strokes) {
					Slider("Stroke speed", &pv.speed, 0.1f, 3.0f, "%.2f a second");
					Slider("Stroke length", &pv.strokeLen, 0.05f, 0.8f, "%.2f of the depth");
				}
				Slider("Enters higher", &pv.up, -2.0f, 2.0f, "%.2f units");
				Slider("Enters to his right", &pv.side, -2.0f, 2.0f, "%.2f units");
				Slider("Tip up", &pv.pitch, -40.0f, 40.0f, "%.0f degrees");
				Slider("Tip to his right", &pv.yaw, -40.0f, 40.0f, "%.0f degrees");
				Help("Places and angles the test schlong in his mouth (negative = lower / to his left / tip down). The lips, cheeks and throat fit it where it is, the same as a real one.");
				if (ImGuiMCP::Button(T("Centre it"))) { pv.up = pv.side = pv.pitch = pv.yaw = 0.0f; }
				Check("Keep it in the middle of the open mouth", &pv.autoCentre);
				Help("The jaw opens downward, so the middle of his lips at rest ends up high in the open mouth: this lowers the test schlong by half of how far the jaw has dropped, keeping it in the middle of the opening. \"Enters higher\" moves it from there.");
				Slider("Tilt my head back", &pv.tilt, 0.0f, 45.0f, "%.0f degrees");
				Help("Tilts the player's head back (chin up) while testing, so the throat bulge is easier to see.");
			}

		}

		void __stdcall DrawInfo()
		{
			auto& c = Config::get();
			bool  ch = false;
			ImGuiMCP::TextWrapped("%s", T("DeepFit moves the mouth of SMP Head (High Poly Head) actors to fit the shaft in it. Changes apply at once and are saved to DeepFit.ini."));
			ch |= Check("Enabled", &c.enabled);
			bool women = !c.malesOnly;
			if (Check("Women's heads too", &women)) {
				c.malesOnly = !women;
				ch = true;
			}
			Help("Also fits women whose head is built for DeepFit (its mouth bones, weights and throat morphs), with their own settings - the \"Women's heads\" drop-down at the bottom of each page, saved to DeepFit_Female.ini. Off: only men's mouths.");
			ch |= Check("Guess unknown skeletons", &c.guessRigs);
			Help("A partner whose skeleton DeepFit does not know: his schlong bones are found from their names (Penis, Dick, Cock, Genital, Shaft, Knot, or the short \"XD 2\" style many rigs use).");
			ch |= Check("Read his schlong where PPA aims it", &c.readAfterPpa);
			Help("PPA bends his schlong into the mouth after the animation, later in the frame than DeepFit runs. On: DeepFit reads it as the last frame ended (where PPA put it, as you see it). Off: as the animation has it (before PPA's aim) - a scene where it is clearly in his mouth but the mouth does not open needs this on.");
			ch |= Check("Anything at his lips counts", &c.anyMeshAtLips);
			Help("Last resort for a partner where nothing is found at all: any part of his mesh that reaches the lips opens the mouth and fills the throat. Heavier, and his body near the face can be read too - leave it off unless you need it.");
			Section("What DeepFit found");
			{
				std::string race;
				float       ago = 0.0f;
				const std::string st = DeepFit::StatusText(&race, &ago);
				if (st.empty())
					ImGuiMCP::TextWrapped("%s", T("Nothing yet - it shows here once a shaft is in a fitted mouth."));
				else {
					ImGuiMCP::TextWrapped("%s", st.c_str());
					if (ago > 1.0f)
						ImGuiMCP::TextDisabled(T("(%.0f s ago)"), ago);
				}
				Help("How the last partner was read: a skeleton DeepFit knows, bones it guessed from their names (listed), or anything at his lips; and whether the shaft came from his mesh or was built round his bones. If a guessed one looks wrong, this is why.");
				if (!race.empty()) {
					float k = c.raceStrengthFor(race);
					char  lbl[160];
					std::snprintf(lbl, sizeof(lbl), "%s %s", T("Strength for"), race.c_str());
					if (ImGuiMCP::SliderFloat(lbl, &k, 0.2f, 2.0f, "x%.2f")) {
						c.setRaceStrength(race, k);
						ch = true;
					}
					Help("Scales how wide the mouth opens and how much the throat bulges for every partner of this race (1 = as the sliders below say). Saved per race in DeepFit.ini (RaceStrength).");
				}
				if (!c.raceStrength.empty()) {
					ImGuiMCP::TextDisabled("%s %s", T("Races set:"), c.raceStrength.c_str());
					ImGuiMCP::SameLine();
					if (ImGuiMCP::Button(T("Clear races"))) {
						c.clearRaces();
						ch = true;
					}
				}
			}

			Section("Heads");
			{
				const std::string mine = DeepFit::PlayerHeadText();
				if (!mine.empty())
					ImGuiMCP::TextWrapped("%s", mine.c_str());
				auto& heads = Heads::All();
				if (heads.empty())
					ImGuiMCP::TextDisabled("%s", T("No head profiles - every head uses DeepFit's own bones and the settings above."));
				for (auto& h : heads) {
					std::string who;
					for (auto& r : h.races) who += (who.empty() ? "" : ", ") + r;
					for (auto& m : h.meshes) who += (who.empty() ? "" : ", ") + m;
					ImGuiMCP::BulletText("%s - %s (%zu %s)", h.name.c_str(), who.empty() ? "-" : who.c_str(), h.bones.size(), T("bones set"));
				}
				if (ImGuiMCP::Button(T("Reload head files")))
					DeepFit::RequestHeadReload();
				ImGuiMCP::SameLine();
				if (ImGuiMCP::Button(T("Refresh my head")))
					DeepFit::RequestPlayerRefresh();
				Help("After changing race or head (RaceMenu): DeepFit lets go of your old head and finds your new one's bones, head profile and teeth again at once - no scene needed.");
				Help("Each head DeepFit fits can have a profile: a file in Data\\SKSE\\Plugins\\DeepFit\\Heads. It names the races (or head meshes) it is for, the head's bones and which way each pushes, and keeps that head's own settings (the drop-downs at the bottom of each page). To add a head, copy one of the files, change it and press Reload - nothing needs compiling. Heads without a profile use the settings above.");
			}

			Section("Presets");
			ImGuiMCP::Text("%s", T("Men's heads"));
			if (PresetsFor(c, false))
				ch = true;
			ImGuiMCP::Text("%s", T("Women's heads"));
			{
				auto& f = Config::female();
				if (PresetsFor(f, true))
					f.save();
			}

			Section("Schlong shapes");
			if (DeepFit::AnatomicaFound()) {
				ImGuiMCP::TextWrapped("%s", T("Anatomica is installed: it tells DeepFit each schlong's shape (its compatibility files, menu choices and More Nasty Critters), so DeepFit's own detection stays off and the two never disagree."));
			} else {
				ch |= Check("Find each schlong's shape", &c.detectShapes);
				Help("Without Anatomica, DeepFit finds the partner's schlong shape itself: a keyword on the schlong (DeepFit_Shape_Flared / _Knotted / _Ridged / _Segmented / _Tentacle / _Plain, or Anatomica's Anatomica_Shape_ ones), its name (horse, canine, knot, argonian, ...), then his race. What it found shows under \"What DeepFit found\".");
			}

			Section("Other mods");
			ImGuiMCP::TextWrapped("%s", T("Other mods can switch Mega mouth with a mod event: SendModEvent(\"DeepFit_MegaMouth\", \"\", 1.0) on, 0.0 off, -1.0 back to the setting here. \"DeepFit_Mouth\" the same for DeepFit opening the mouth at all."));
			ch |= Check("Debug log", &c.debugLog);
			ImGuiMCP::Separator();
			if (ImGuiMCP::Button(T("Reset to defaults")))
				c.resetDefaults();
			Help("Every setting on this page back to as installed (your Mega mouth key and the debug log stay). Each slider's default sits in the middle of its range.");
			ImGuiMCP::SameLine();
			if (ImGuiMCP::Button(T("Reset women's heads to defaults")))
				Config::female().resetDefaults();
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
		// 0.4.4: a page each, in this order
		SKSEMenuFramework::SetSection("DeepFit");
		SKSEMenuFramework::AddSectionItem("Lips", DrawLips);
		SKSEMenuFramework::AddSectionItem("Mouth", DrawMouthPage);
		SKSEMenuFramework::AddSectionItem("Face", DrawFace);
		SKSEMenuFramework::AddSectionItem("Throat", DrawThroat);
		SKSEMenuFramework::AddSectionItem("Sounds", DrawSounds);
		SKSEMenuFramework::AddSectionItem("Testing", DrawTesting);
		SKSEMenuFramework::AddSectionItem("Info", DrawInfo);
		logs::info("DeepFit: settings pages added to the SKSE Menu Framework");
	}
}
