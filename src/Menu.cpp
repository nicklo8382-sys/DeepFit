#include "Menu.h"
#include "Config.h"
#include "SKSEMenuFramework.h"
#include "DeepFit.h"

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

		bool Check(const char* label, bool* v) { return ImGuiMCP::Checkbox(T(label), v); }
		bool Slider(const char* label, float* v, float lo, float hi, const char* fmt) { return ImGuiMCP::SliderFloat(T(label), v, lo, hi, T(fmt)); }
		void Section(const char* t) { ImGuiMCP::SeparatorText(T(t)); }

		// 0.4.0: presets - the throat and mouth sliders set together
		void Preset(Config& c, int which)
		{
			const Config d{};
			c.throatShapeGain = d.throatShapeGain; c.throatDetail = d.throatDetail; c.throatShapeMax = d.throatShapeMax;
			c.throatWideExtra = d.throatWideExtra; c.throatMotion = d.throatMotion; c.mouthWideGain = d.mouthWideGain; c.lipRound = d.lipRound;
			c.cheekGain = d.cheekGain; c.gulpSize = d.gulpSize; c.shapeRidge = d.shapeRidge; c.shapeKnob = d.shapeKnob;
			if (which == 0) {   // subtle
				c.throatShapeGain = 1.0f; c.throatDetail = 1.2f; c.throatShapeMax = 2.0f; c.throatWideExtra = 0.1f; c.throatMotion = 0.2f;
				c.mouthWideGain = 0.8f; c.lipRound = 0.7f; c.cheekGain = 0.6f; c.gulpSize = 0.8f; c.shapeRidge = 0.4f; c.shapeKnob = 0.5f;
			} else if (which == 2) {   // extreme
				c.throatShapeGain = 2.4f; c.throatDetail = 2.2f; c.throatShapeMax = 3.5f; c.throatWideExtra = 0.8f; c.throatMotion = 1.0f;
				c.mouthWideGain = 1.4f; c.lipRound = 0.4f; c.cheekGain = 1.5f; c.gulpSize = 1.8f; c.shapeRidge = 0.9f; c.shapeKnob = 1.0f;
			}
		}

		void __stdcall DrawMouth()
		{
			auto& c = Config::get();
			bool  ch = false;
			ImGuiMCP::TextWrapped("%s", T("DeepFit moves the mouth of SMP Head (High Poly Head) actors to fit the shaft in it. Changes apply at once and are saved to DeepFit.ini."));
			ch |= Check("Enabled", &c.enabled);
			ch |= Check("Men only", &c.malesOnly);
			Help("Only men's mouths are fitted. Off: any head with the SMP Head mouth bones - it needs a head built for DeepFit (its mouth bones, weights and throat morphs) to look right.");
			ch |= Check("Guess unknown skeletons", &c.guessRigs);
			Help("A partner whose skeleton DeepFit does not know: his schlong bones are found from their names (Penis, Dick, Cock, Genital, Shaft, Knot, or the short \"XD 2\" style many rigs use).");
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

			Section("Presets");
			if (ImGuiMCP::Button(T("Subtle"))) { Preset(c, 0); ch = true; }
			ImGuiMCP::SameLine();
			if (ImGuiMCP::Button(T("Default"))) { Preset(c, 1); ch = true; }
			ImGuiMCP::SameLine();
			if (ImGuiMCP::Button(T("Extreme"))) { Preset(c, 2); ch = true; }
			Help("Sets the throat, mouth-width, cheek, gulp and shape sliders together. Fine-tune them below afterwards.");

			Section("Test (no scene needed)");
			auto& pv = DeepFit::GetPreview();
			Check("Show a test schlong in my mouth", &pv.on);
			Help("A made-up schlong goes into the player's mouth, so you can tune the mouth and throat here without starting a scene. Not saved - switch it off when done. Needs a head built for DeepFit on the player.");
			if (pv.on) {
				Check("Show it", &pv.show);
				Help("A plain schlong is shown in his mouth where the test one is (only its thickness and depth - the shapes below are felt by the mouth and throat, not drawn).");
				Slider("Tip depth", &pv.depth, 0.5f, 16.0f, "%.1f units past the lips");
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
					Help("A plain schlong the size of one of TNG's size categories (scaled as set in TNG's MCM), or of the player's own TNG size. Sliders: the height, width and shape below.");
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
				Slider("Tilt my head back", &pv.tilt, 0.0f, 45.0f, "%.0f degrees");
				Help("Tilts the player's head back (chin up) while testing, so the throat bulge is easier to see.");
			}

			Section("Mouth");
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
			ch |= Slider("Room round the shaft", &c.mouthExtra, 0.0f, 6.0f, "%.2f");
			Help("The mouth opens this much more than the shaft is thick.");
			ch |= Slider("Jaw (units per 100)", &c.jawUnits, 1.0f, 20.0f, "%.1f");
			Help("How far the jaw morph at 100 opens the jaw. Lower = the jaw opens wider for the same shaft.");
			ch |= Slider("Lips round (units per 100)", &c.lipUnits, 1.0f, 16.0f, "%.1f");
			ch |= Slider("Corners spread (units per 100)", &c.widenUnits, 0.5f, 10.0f, "%.1f");
			ch |= Slider("Most a morph is set to", &c.morphMax, 10.0f, 600.0f, "%.0f");
			Help("MFEE caps the morphs too: its Max in MuFacialExpressionExtended.ini (100 as installed). Raise both for more than the morph's full size.");

			Section("Lips");
			ch |= Slider("Stretch past the shaft", &c.stretch, -1.0f, 1.0f, "%.2f");
			ch |= Slider("Close in round a slim shaft", &c.lipCloseMax, 0.0f, 4.0f, "%.2f");
			ch |= Slider("Upper lip, at most", &c.upperLipMax, 0.0f, 3.0f, "%.1f");
			ch |= Slider("Upper lip down", &c.upperLipDown, 0.0f, 1.2f, "%.2f");
			ch |= Slider("Lower lip up", &c.lowerLipUp, 0.0f, 1.2f, "%.2f");
			Help("While the mouth is full the lower lip is pushed up this far (its middle fully, the sides a little less), closer round the shaft.");
			ch |= Check("Open to the shaft's real thickness", &c.lipsAlongShaft);
			Help("The mouth opens to the shaft measured straight across it. Off: measured straight up and down his face, so a shaft going in at an angle opens it wider (as before 0.4.1).");
			Help("While the mouth is full the upper lip is pulled down this far (its middle fully, the sides a little less) - the lip-round morph peaks it up towards the nose.");
			ch |= Slider("Lower lip, at most", &c.lowerLipMax, 0.0f, 4.5f, "%.1f");
			ch |= Slider("Corners, at most", &c.cornerMax, 0.0f, 2.5f, "%.1f");
			ch |= Slider("Mouth opens to the sides", &c.mouthWideGain, 0.0f, 2.5f, "x%.2f");
			Help("The corners of his mouth pull out to the sides to the schlong's width (DeepFit's DF_MouthWide morph), so the mouth opens wide round it instead of puckering.");
			ch |= Slider("Corners out, at most", &c.mouthWideMax, 0.0f, 3.0f, "%.1f units");
			ch |= Slider("Corners out with the jaw", &c.cornerOpen, 0.0f, 0.4f, "%.2f");
			ch |= Slider("Lips round forward", &c.lipRound, 0.0f, 1.0f, "%.2f");
			Help("How much the lips push forward into an O. Lower = less of a pucker, more of a wide-open mouth (1 = as before 0.3.8).");
			ch |= Slider("Cheek bulge", &c.cheekGain, 0.0f, 3.0f, "x%.1f");
			Help("A shaft pushed to one side of his mouth bulges that cheek out (SMP Head's cheek bones). 0 = off.");
			ch |= Slider("Cheek, at most", &c.cheekMax, 0.0f, 3.0f, "%.1f units");
			ch |= Slider("Room before the cheek", &c.cheekInner, 0.5f, 3.0f, "%.1f units");
			ch |= Slider("Lips drag with the stroke", &c.lipDrag, 0.0f, 1.0f, "%.2f units");
			Help("His lips roll out a little as the shaft pulls back and tuck in as it pushes in. 0 = off.");
			ch |= Slider("Stroke speed for full drag", &c.lipDragSpeed, 5.0f, 60.0f, "%.0f units/s");

			Section("Face");
			ch |= Check("His face reacts", &c.faceReact);
			Help("His eyes close and his brows lift the deeper it goes; in a gag his eyes clench and his brows knit. Goes over OStim's own expression, never below it.");
			if (c.faceReact) {
				ch |= Slider("Eyes close with depth", &c.faceEyes, 0.0f, 1.0f, "%.2f");
				ch |= Slider("Brows lift with depth", &c.faceBrows, 0.0f, 1.0f, "%.2f");
				ch |= Slider("Clench in a gag", &c.faceGag, 0.0f, 1.0f, "%.2f");
			}

			Section("After he pulls out");
			ch |= Slider("Mouth stays open for", &c.recoverHold, 0.0f, 3.0f, "%.1f s");
			Help("After the shaft leaves his mouth it stays a little open for a moment before it closes. 0 = closes straight away.");
			ch |= Slider("How open it stays", &c.recoverOpen, 0.0f, 1.0f, "%.2f");
			ch |= Check("Then he swallows", &c.recoverGulp);

			Section("Throat and teeth");
			ch |= Check("Throat bulge, by the schlong's shape", &c.throatShape);
			Help("DeepFit's own throat morphs: ten thin bands down the front of his throat, from under his chin to just above the neck seam. Where the shaft is, the throat fills out to a straight line from chin to neck, then swells past it by how thick that part of the schlong is, so a flared head, a medial ring or a knot shows as it goes down.");
			if (c.throatShape) {
				ch |= Slider("Throat starts at", &c.throatFrom, 0.0f, 12.0f, "%.1f units deep");
				Help("How far past his lips the schlong has to be to reach the top of his throat (under his chin).");
				ch |= Slider("Shaft per band", &c.throatBand, 0.3f, 2.0f, "%.2f units");
				Help("How much of the shaft's length each of the ten bands stands for (smaller = the throat shows a shorter stretch of it, in more detail).");
				ch |= Slider("Fill out to the line", &c.throatFill, 0.0f, 1.0f, "%.2f");
				ch |= Slider("Room in an empty throat", &c.throatRest, 0.0f, 2.0f, "%.1f");
				Help("A schlong thinner than this (half its thickness, in units) passes without a bulge.");
				ch |= Slider("Bulge per thickness", &c.throatShapeGain, 0.2f, 4.0f, "x%.1f");
				ch |= Slider("Shape stands out", &c.throatDetail, 1.0f, 3.0f, "x%.1f");
				Help("A flared head, medial ring or knot this many times more pronounced than the shaft itself (1 = exactly as the schlong is shaped).");
				ch |= Slider("Bulge, at most", &c.throatShapeMax, 0.5f, 5.0f, "%.1f units");
				ch |= Slider("Wider", &c.throatWideExtra, 0.0f, 1.0f, "%.2f");
				Help("Every bulge spreads further across his neck as soft shoulders round the shaft's own ridge (0 = as wide as the shaft, 1 = most of the front of his neck).");
				ch |= Slider("Moves with each stroke", &c.throatMotion, 0.0f, 1.5f, "%.2f units");
				Help("A soft ripple that slides along his throat as the shaft goes in and out, so it still moves when his throat is as full as it gets.");
				ch |= Slider("Ripple length", &c.throatMotionLen, 0.5f, 6.0f, "%.1f units");
				ch |= Slider("Spreads fully sideways at", &c.throatWideUnits, 0.3f, 3.0f, "%.1f units wider");
				Help("A schlong wider than it is tall spreads the bulge out to the sides; this much wider (half width over half height) spreads it all the way.");
				ch |= Check("Measure along the shaft", &c.throatAxisFit);
				Help("Measures the shaft along its own line, so one that goes in at an angle or bends reads its real thickness. Off: sliced straight back from his lips.");
				ch |= Slider("Ridged schlong's ridge", &c.shapeRidge, 0.0f, 2.0f, "%.2f units");
				ch |= Slider("Segmented schlong's knobs", &c.shapeKnob, 0.0f, 2.0f, "%.2f units");
				Help("Anatomica tells DeepFit when the schlong is ridged or segmented (the same shape its belly bulge uses); its ridge or knobs then show down his throat.");
			}
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
			ch |= Check("Teeth part while the mouth is full", &c.teethOpen);
			{
				static const char* kTeeth[] = { "Off", "While the mouth is full", "Always (kept moved)" };
				ImGuiMCP::Text("%s", T("Move the teeth:"));
				for (int i = 0; i < 3; ++i) {
					ImGuiMCP::SameLine();
					if (ImGuiMCP::RadioButton(T(kTeeth[i]), c.teethMove == i)) {
						c.teethMove = i;
						ch = true;
					}
				}
				Help("Moves his teeth a little down and back so they sit clear of the shaft. They are hung on a node of DeepFit's own under his head, so the game rebuilding his face does not undo it for long - DeepFit checks once a second and puts them back. Always: kept moved outside scenes too, only on the men DeepFit fits, and only when this is chosen (it costs next to nothing).");
				if (c.teethMove > 0) {
					ch |= Slider("Teeth down", &c.teethDown, -1.0f, 2.0f, "%.2f units");
					ch |= Slider("Teeth back", &c.teethBack, -1.0f, 2.0f, "%.2f units");
				}
			}

			Section("Other mods");
			ImGuiMCP::TextWrapped("%s", T("Other mods can switch Mega mouth with a mod event: SendModEvent(\"DeepFit_MegaMouth\", \"\", 1.0) on, 0.0 off, -1.0 back to the setting here. \"DeepFit_Mouth\" the same for DeepFit opening the mouth at all."));
			ch |= Check("Debug log", &c.debugLog);
			ImGuiMCP::Separator();
			if (ImGuiMCP::Button(T("Reset to defaults")))
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
