#include <unordered_set>
#include <functional>
#include <cctype>
#include <mutex>
#include <random>
#include <filesystem>
#include "DeepFit.h"
#include "Config.h"
#include "Heads.h"
#include <iterator>

// ---------------------------------------------------------------------------------------------
// DeepFit
//
// Every frame, for every loaded actor that has the SMP Head mouth bones, find the schlong meshes
// of nearby actors, skin their vertices on the CPU (so TNG scaling/bending and the real mesh
// shape are respected), and push the lip / corner / nose / throat bones straight out to the
// shaft surface.  No physics: the bones are placed where the shaft is, every frame.
// ---------------------------------------------------------------------------------------------

namespace
{
	using namespace RE;

	enum class Kind : std::uint8_t
	{
		Lip,
		Corner,
		Nose,
		Throat,
		Cheek,  // 0.4.0
		Jaw     // 0.5.1: a beast's whole lower jaw (a head profile's "Lower Jaw" bone) - drops as far as the front of the lower lip has to
	};

	struct BoneSpec
	{
		const char* suffix;  // bone name (FSMP may prefix it with "hdtSSEPhysics_AutoRename_Head_XXXXXXXX ")
		Kind        kind;
		NiPoint3    dir;     // push direction in head space (X right, Y forward, Z up)
	};

	// Nif convention of this head: "Left" bones sit at negative X.
	const BoneSpec kSpecs[] = {
		{ "Upper Lip Center", Kind::Lip, { 0.0f, 0.0f, 1.0f } },
		{ "Upper Lip Left", Kind::Lip, { -0.6f, 0.0f, 0.8f } },
		{ "Upper Lip Right", Kind::Lip, { 0.6f, 0.0f, 0.8f } },
		{ "Bottom Lip Center", Kind::Lip, { 0.0f, 0.0f, -1.0f } },
		{ "Bottom Lip Left", Kind::Lip, { -0.6f, 0.0f, -0.8f } },
		{ "Bottom Lip Right", Kind::Lip, { 0.6f, 0.0f, -0.8f } },
		{ "Corner Lip Left", Kind::Corner, { -1.0f, 0.0f, 0.0f } },
		{ "Corner Lip Right", Kind::Corner, { 1.0f, 0.0f, 0.0f } },
		{ "Nose Base", Kind::Nose, { 0.0f, 0.0f, 1.0f } },
		{ "Throat Center 1", Kind::Throat, { 0.0f, 1.0f, 0.0f } },
		{ "Throat Center 2", Kind::Throat, { 0.0f, 1.0f, 0.0f } },
		{ "Throat Center 3", Kind::Throat, { 0.0f, 1.0f, 0.0f } },
		{ "Cheeck Side L", Kind::Cheek, { -1.0f, 0.0f, 0.0f } },   // 0.4.0: a shaft pushed to one side bulges that cheek (SMP Head's spelling)
		{ "Cheeck Side R", Kind::Cheek, { 1.0f, 0.0f, 0.0f } },
		// beast-race heads (Argonian / Khajiit): just in front of each corner, above and below the mouth line, and the side of
		// the lower jaw. Heads without them skip them; a head profile can change any bone's direction or add its own bones.
		{ "Muzzle Upper Left", Kind::Lip, { -0.3f, 0.0f, 0.95f } },
		{ "Muzzle Upper Right", Kind::Lip, { 0.3f, 0.0f, 0.95f } },
		{ "Muzzle Lower Left", Kind::Lip, { -0.3f, 0.0f, -0.95f } },
		{ "Muzzle Lower Right", Kind::Lip, { 0.3f, 0.0f, -0.95f } },
		{ "Muzzle Jaw Left", Kind::Lip, { -0.3f, 0.0f, -0.95f } },   // the middle of the lower jaw's side
		{ "Muzzle Jaw Right", Kind::Lip, { 0.3f, 0.0f, -0.95f } },
	};
	constexpr int kUpperCenter = 0;
	constexpr int kBottomCenter = 3;
	constexpr int kNumSpecs = static_cast<int>(sizeof(kSpecs) / sizeof(kSpecs[0]));
	constexpr int kMaxSpecs = 48;   // 0.4.6: DeepFit's own bones plus what a head profile adds
	// 0.5.6: a rigged mouth with 24 bones: bones 8-23 copy how far each of these of his head's own bones moved, so the
	// edge of the mouth (where it meets his lips, cheeks and jaw) moves exactly as his lips do - with the two averaged
	// lip bones the palate's front came away from his upper lip and the floor of his mouth stuck out under his lower lip
	const char* const kMouthRim[] = { "Upper Lip Center", "Upper Lip Left", "Upper Lip Right", "Bottom Lip Center", "Bottom Lip Left",
		"Bottom Lip Right", "Corner Lip Left", "Corner Lip Right", "Muzzle Upper Left", "Muzzle Upper Right", "Muzzle Lower Left",
		"Muzzle Lower Right", "Muzzle Jaw Left", "Muzzle Jaw Right", "Cheeck Side L", "Cheeck Side R" };
	constexpr int kMouthRimN = static_cast<int>(sizeof(kMouthRim) / sizeof(kMouthRim[0]));
	constexpr int kMouthNodes = 6 + kMouthRimN;
	constexpr int  kTeethLowerSlot = 8 + kMouthRimN;   // 0.5.9: a 25-bone mouth's lower teeth (bone 24)
	constexpr int  kThroatSlot0 = 9 + kMouthRimN;     // 0.5.9: a 30-bone mouth's throat bones (25-29)
	constexpr int  kThroatN = 5;
	constexpr bool rigMouthBones(std::uint32_t nb) { return nb == 6 || nb == 8 || nb == 8 + kMouthRimN || nb == 9 + kMouthRimN || nb == 9 + kMouthRimN + kThroatN; }

	// 0.5.4: the lips are mostly moved by DeepFit's bones, not its morphs - a beard following only the morphs stayed where
	// the lips were pushed out of it (the moustache sank into the upper lip, the chin beard into the lower). Each lip bone
	// also gets two morphs in the beard's .tri - the beard over it moved 1 unit out along the bone's push (P) and in (N),
	// scaled to kBeardBoneUnits at 100 - and DeepFit sets them as the bone moves
	constexpr float kBeardBoneUnits = 5.0f;
	const std::string& beardBoneName(int a_spec, bool a_in)
	{
		static std::vector<std::string> names = [] {
			std::vector<std::string> v;
			for (int i = 0; i < kMaxSpecs; ++i) {
				v.push_back("DF_BeardBone" + std::to_string(i) + "P");
				v.push_back("DF_BeardBone" + std::to_string(i) + "N");
			}
			return v;
		}();
		return names[a_spec * 2 + (a_in ? 1 : 0)];
	}

	// 0.4.6: the bones one head has: DeepFit's own list, then its head profile's changes (direction / kind) and additions
	struct HeadSpec
	{
		std::string name;
		Kind        kind;
		NiPoint3    dir;
	};
	std::vector<HeadSpec> specsFor(int profile)
	{
		std::vector<HeadSpec> v;
		for (auto& k : kSpecs)
			v.push_back({ k.suffix, k.kind, k.dir });
		auto& heads = Heads::All();
		if (profile < 0 || profile >= static_cast<int>(heads.size()))
			return v;
		for (auto& b : heads[profile].bones) {
			std::string kk = b.kind;
			for (auto& c : kk)
				c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
			const Kind kind = kk == "corner" ? Kind::Corner : kk == "nose" ? Kind::Nose : kk == "throat" ? Kind::Throat : kk == "cheek" ? Kind::Cheek : kk == "jaw" ? Kind::Jaw : Kind::Lip;
			const NiPoint3 d(b.x, b.y, b.z);
			auto it = std::find_if(v.begin(), v.end(), [&](const HeadSpec& h) { return h.name == b.name; });
			if (it != v.end()) {
				it->kind = kind;
				it->dir = d;
			} else if (static_cast<int>(v.size()) < kMaxSpecs)
				v.push_back({ b.name, kind, d });
		}
		return v;
	}

	struct DrivenBone
	{
		NiPointer<NiAVObject> node;
		NiPointer<NiNode>     parent;
		NiPoint3              restLocal{};
		float                 cur = 0.0f;  // current push along dir
		bool                  valid = false;
	};

	// 0.2.0: the mouth morphs DeepFit drives (SMP Head's, through MFEE)
	// 0.3.5: + DeepFit's own throat morphs (added to SMP Head's PPA_HPH_Male.tri), ten thin bands down the front of his
	// throat along the straight line from under his chin to the neck seam: DF_ThroatFill1-10 bring the band's skin out onto
	// that line (rounded across, like a tube under the skin), DF_Throat1-10 push it 1 unit further out at 100
	constexpr int         kBands = 10;
	constexpr int         kFill0 = 6, kBand0 = kFill0 + kBands, kWide0 = kBand0 + kBands;   // 0.3.5: + DF_ThroatWide1-10 (a wide shaft spreads it sideways)
	constexpr int         kWideT0 = kWide0 + kBands;   // 0.3.6: + DF_ThroatWideT1-10 (the swelling spread across that width too)
	constexpr int         kMouthWide = kWideT0 + kBands;   // 0.3.8: DF_MouthWide - the corners pulled out to the sides (a wide-open mouth, not a pucker)
	constexpr int         kRidge0 = kMouthWide + 1;        // 0.4.0: DF_ThroatRidge1-10 - a ridged schlong's ridge down the middle
	constexpr int         kKnob0 = kRidge0 + kBands;       // 0.4.0: DF_ThroatKnob1-10 - a segmented one's knobs, one each side
	constexpr int         kLowerRound = kKnob0 + kBands;   // 0.4.4: DF_LowerLipRound - the lower lip's sides out and down (a round opening)
	constexpr int         kUpperJaw = kLowerRound + 1;   // 0.4.7: DF_UpperJawDown - a beast's upper snout tilted down onto the shaft
	constexpr int         kMorphs = kUpperJaw + 1;
	constexpr const char* kMorphNames[kMorphs] = { "PPA_JawWide", "PPA_LipStretch", "PPA_LipWiden", "PPA_TH1", "PPA_TH2", "PPA_TH3",
		"DF_ThroatFill1", "DF_ThroatFill2", "DF_ThroatFill3", "DF_ThroatFill4", "DF_ThroatFill5",
		"DF_ThroatFill6", "DF_ThroatFill7", "DF_ThroatFill8", "DF_ThroatFill9", "DF_ThroatFill10",
		"DF_Throat1", "DF_Throat2", "DF_Throat3", "DF_Throat4", "DF_Throat5",
		"DF_Throat6", "DF_Throat7", "DF_Throat8", "DF_Throat9", "DF_Throat10",
		"DF_ThroatWide1", "DF_ThroatWide2", "DF_ThroatWide3", "DF_ThroatWide4", "DF_ThroatWide5",
		"DF_ThroatWide6", "DF_ThroatWide7", "DF_ThroatWide8", "DF_ThroatWide9", "DF_ThroatWide10",
		"DF_ThroatWideT1", "DF_ThroatWideT2", "DF_ThroatWideT3", "DF_ThroatWideT4", "DF_ThroatWideT5",
		"DF_ThroatWideT6", "DF_ThroatWideT7", "DF_ThroatWideT8", "DF_ThroatWideT9", "DF_ThroatWideT10",
		"DF_MouthWide",
		"DF_ThroatRidge1", "DF_ThroatRidge2", "DF_ThroatRidge3", "DF_ThroatRidge4", "DF_ThroatRidge5",
		"DF_ThroatRidge6", "DF_ThroatRidge7", "DF_ThroatRidge8", "DF_ThroatRidge9", "DF_ThroatRidge10",
		"DF_ThroatKnob1", "DF_ThroatKnob2", "DF_ThroatKnob3", "DF_ThroatKnob4", "DF_ThroatKnob5",
		"DF_ThroatKnob6", "DF_ThroatKnob7", "DF_ThroatKnob8", "DF_ThroatKnob9", "DF_ThroatKnob10",
		"DF_LowerLipRound", "DF_UpperJawDown" };

	// 0.4.4: DF_Throat / DF_ThroatWideT / DF_ThroatRidge / DF_ThroatKnob are 3 units at 100 in the head (were 1): every
	// value DeepFit works out for them (in units x 100) is sent as a third, so a big bulge no longer hits MFEE's limit of
	// 100 (all shapes looked the same, flattened at 1 unit)
	constexpr float kThroatScale = 3.0f;
	constexpr bool throatScaled(int m)
	{
		return (m >= kBand0 && m < kBand0 + kBands) || (m >= kWideT0 && m < kWideT0 + kBands) || (m >= kRidge0 && m < kRidge0 + kBands) ||
		       (m >= kKnob0 && m < kKnob0 + kBands);
	}

	struct Receiver
	{
		NiAVObject*           root = nullptr;  // identity of the 3D we cached from
		NiPointer<NiAVObject> head;
		NiPointer<NiAVObject> hmag;          // 0.1.3: "NPC Head MagicNode [Hmag]" - in front of his face
		NiPoint3              fx{ 1, 0, 0 }, fy{ 0, 1, 0 }, fz{ 0, 0, 1 };   // 0.1.3: his face's right / forward / up, this frame
		bool                  toldFrame = false;
		DrivenBone            bones[kMaxSpecs];
		std::vector<HeadSpec> specs;              // 0.4.6: this head's bones (DeepFit's list + its head profile)
		int                   profile = -1;       // 0.4.6: its head profile (-1 none)
		int                   headsGen = -1;
		double                skinCheckAt = -1e9;   // 0.4.9: when his lip bones were last checked against his face's skin
		int                   nspecs() const { return static_cast<int>(specs.size()); }
		ActorHandle           actor;                 // 0.2.0: whose face the morphs go on
		float                 morph[kMorphs] = {};   // 0.2.0: eased mouth morph values (0-100)
		int                   sent[kMorphs];   // set to -1 (never sent) in the constructor
		int                   beardSent[kMaxSpecs * 2];   // 0.5.4: the beard's bone morphs (out / in for each lip bone), as sent
		std::uint64_t         beardBones = 0;             // 0.5.4: the lip bones his beards have morphs for (DF_BeardBone<i>P / N)
		Receiver()
		{
			std::fill(std::begin(sent), std::end(sent), -1);
			std::fill(std::begin(beardSent), std::end(beardSent), -1);
		}
		float                 sendTimer = 0.0f;
		bool                  teeth = false;
		bool                  teethSent = false;
		bool                  toldMouth = false;
		int                   found = 0;
		bool                  checked = false;
		float                 retry = 0.0f;  // seconds until we look for the mouth bones again
		int                   shape = 0;     // 0.4.0: the shape family of the schlong in his mouth (Anatomica's, by mod event; 6 ridged, 7 segmented)
		double                shapeAt = -1e9;
		double                gulpAt = -1e9, gagAt = -1e9;   // 0.4.0: a swallow / a gag (Anatomica's mod events)
		float                 gagK = 1.0f;
		// 0.4.0: how he moves in the mouth (lip drag, the face, gag / gulp by themselves, the mouth after he pulls out)
		bool                  full = false;          // the shaft in his mouth this frame
		float                 depthPrev = -1.0f, vel = 0.0f, drag = 0.0f, dragWant = 0.0f, depthK = 0.0f;
		bool                  wasIn = false, recoverGulped = true;
		double                leftAt = -1e9, extAt = -1e9;   // when it left his mouth; Anatomica's last gag / gulp
		float                 leftJaw = 0.0f, holdT = 0.0f;
		float                 raceK = 1.0f;          // 0.4.0: the partner's race strength
		float                 faceBase[8] = {}, faceSent[8] = { -1, -1, -1, -1, -1, -1, -1, -1 };
		bool                  faceOn = false;
		// 0.4.0: the teeth moved down / back (their head-bone weights hung on a node of DeepFit's own under his head)
		struct TeethHook { NiPointer<BSGeometry> geo; std::uint32_t idx; NiPointer<NiAVObject> orig; };
		std::vector<TeethHook> teethHooks;
		std::vector<NiPointer<NiAVObject>> teethHidden;   // 0.4.6: his teeth / mouth parts hidden for the scene
		std::vector<NiPointer<NiAVObject>> teethBones;    // 0.4.7: "DF Teeth Only" bones in his mouth parts (the teeth alone - the tongue stays)
		float                  hideScan = 0.0f;
		NiPointer<NiNode>      teethNode;
		NiPointer<NiNode>      teethHideNode;            // 0.5.2: the teeth slot of DeepFit's mouths hangs here, shrunk
		std::vector<TeethHook> teethSlots;
		// 0.5.4: DeepFit's mouths (six bones: head, teeth, three lower-jaw ones, tongue) - their lower gums and floor follow
		// his lower jaw bones and the tongue is pushed down by the shaft; the mouth's own bones hang on these nodes of DeepFit's
		NiPointer<NiNode>      mouthNodes[kMouthNodes];
		// 0.5.9: the throat a 30-bone mouth carries on into his neck: its bones' rest centres and radii (head-bone space,
		// read from the mouth's skin), where they are now (off their rest, and how much wider), and the shaft's points
		NiPointer<NiNode>      throatNodes[kThroatN];
		NiPoint3               throatC[kThroatN], throatOff[kThroatN];
		float                  throatR[kThroatN]{}, throatK[kThroatN]{ 1, 1, 1, 1, 1 };
		bool                   throatSet = false;
		std::vector<NiPoint3>  shaftWorld;
		// 0.5.5: a head that opens with a skeleton jaw bone (no lip bones): DeepFit's own lip markers and that bone
		NiPointer<NiNode>      virtLips[2];
		NiPointer<NiAVObject>  jawBone;
		NiMatrix3              jawBase, jawSet;
		bool                   jawSetValid = false;
		float                  jawBoneDeg = 0.0f;
		float                  jawSign = 0.0f;   // jaw front / mid / back, tongue, 0.5.5: upper lip, lower lip
		bool                   tongueLicks = false;   // 0.5.5: the shaft is in his mouth - the tongue may lick
		std::vector<TeethHook> mouthSlots;
		float                  tongueWant = 0.0f, tongueCur = 0.0f, mouthScan = 0.0f, tongueFwd = 0.0f;
		float                  teethK = 0.0f, teethScan = 0.0f;
		NiPoint3               mouthC{ 0, 0, 0 };        // 0.4.0: the middle of his lips (world), and the test schlong's
		float                  previewD = 0.0f, previewR = 1.3f;   // tip depth and thickness this frame
		int                    previewShape = 0;
		bool                   tiltSet = false;          // 0.4.1: the test page's head tilt
		NiMatrix3              tiltBase, tiltLast;
		bool                   teethTold = false;
		bool                   axesOk = false;
		NiPoint3 dir(int i) const { return specs[i].dir; }
		bool                   inScene = false;
		float                  extraCur = 0.0f, prevVel = 0.0f;   // 0.4.4: an extra push (how much deeper it goes now)
		float                  lastH = 0.0f, lastW = 0.0f;         // 0.4.5: the shaft at his lips, a moment ago
		double                 lastThroughAt = -1e9;
		double                 extraUntil = -1e9;
		bool                   shapeFromAnatomica = false;   // 0.4.4: the shape came from Anatomica (else DeepFit found it)
		float                  knotOpen = 0.0f;
		bool                   female = false;   // 0.4.4: a woman's head built for DeepFit - her own settings
		// 0.5.0: his beards follow his mouth
		struct BeardHead
		{
			NiPointer<BSGeometry>  geo;                 // his face mesh
			NiTransform            bind;                // its mesh -> head bone
			std::vector<NiPoint3>  rest;                // its vertices when looked at (mesh space)
			std::string            tri;
			std::vector<NiPoint3>  morph[kMorphs];      // DeepFit's morphs on it (at 100), where its .tri files have them
			struct BW { std::int8_t spec[4]{}; float w[4]{}; int n = 0; };
			std::vector<BW>        bw;                  // DeepFit's bones on each of its vertices
			int                    weighted = 0;
		};
		struct Beard
		{
			std::string                name, state;
			NiPointer<BSGeometry>      geo;
			std::uint32_t              n = 0;
			std::vector<std::uint32_t> hv;     // the three face vertices under each beard vertex ...
			std::vector<float>         hw;     // ... and how much each counts
			std::vector<float>         dist;   // how far it is from the face
			NiMatrix3                  toBeard;
			float                      toBeardK = 1.0f;
			NiMatrix3                  toBeardRot;          // 0.5.4: head bone -> beard mesh
			float                      toBeardHeadK = 1.0f;
			bool                       culled = false;
		};
		BeardHead              beardHead;
		std::vector<Beard>     beards;
		float                  beardScan = 0.0f;
		double                 gulpSounded = -1e9, gagSounded = -1e9, gaspSounded = -1e9;   // 0.4.4    // 0.4.2: a partner's schlong near him this frame (in a scene)   // 0.4.2: fx/fy/fz found from his mouth (until then his head bone's axes)
	};

	// 0.4.6: the settings this head uses: its head profile's, else the women's / the main set
	// 0.4.8: how far his lower lip drops with the jaw fully open (the head profile's JawDrop; 4.1 on the human head)
	float headJawDrop(const Receiver& r)
	{
		auto& h = Heads::All();
		return (r.profile >= 0 && r.profile < static_cast<int>(h.size())) ? h[r.profile].jawDrop : 4.1f;
	}

	Config& cfgOf(const Receiver& r)
	{
		auto& h = Heads::All();
		return (r.profile >= 0 && r.profile < static_cast<int>(h.size())) ? h[r.profile].cfg : Config::of(r.female);
	}

	struct ShaftVert
	{
		NiPoint3     skinPos;
		std::uint8_t bone[4];
		float        weight[4];
	};

	struct ShaftShape
	{
		NiPointer<BSGeometry>  geom;
		std::vector<ShaftVert> verts;
	};

	struct Partner
	{
		NiAVObject*             root = nullptr;
		std::vector<ShaftShape> shapes;
		float                   age = 0.0f;  // seconds since built
		std::vector<NiPoint3>   world;       // skinned this frame
		bool                    skinnedThisFrame = false;
		int                     looked = 0, genital = 0;  // last scan, for the log
		bool                    told = false;
		// 0.3.1: his schlong's bone line (base to tip) and its thickness. The mesh skinned on the CPU landed far from where
		// the schlong really is in some games (140 units off, so the mouth never found it): each frame the skinned points
		// are checked against the bone line, and when they are off the shaft is built round the bones instead
		std::vector<NiPointer<NiAVObject>> chain;
		// 0.4.7: a bone his mesh names but the game draws it with another one's transform (a creature's parts re-attached
		// to its skeleton) - read where it is drawn
		std::unordered_map<const NiAVObject*, const NiTransform*> drawnBy;
		int                     drawnOther = 0;
		float                   radius = 1.6f;
		int                     method = -1;   // 0 the mesh, 1 the bone line (last frame), for the log
		float                   meshOff = 0.0f;
		bool                    anyMesh = false;   // 0.4.0: nothing recognisable - his whole mesh is read (only what reaches the lips counts)
		std::string             source;            // 0.4.0: how he was read, for the menu
		int                     kind = -1;         // 0.4.4: his schlong's shape, found by DeepFit itself (-1 not looked yet)
		std::string             kindWhy;
		float                   kindAge = 0.0f;
	};

	// his schlong's bones, base to tip
	// 0.4.0: every schlong bone chain DeepFit knows, base to tip (the skeleton rigs OStrands lists, as well as people's
	// and beast forms')
	const std::vector<std::vector<const char*>>& schlongChains()
	{
		static const std::vector<std::vector<const char*>> c{
			{ "NPC Genitals01 [Gen01]", "NPC Genitals02 [Gen02]", "NPC Genitals03 [Gen03]", "NPC Genitals04 [Gen04]", "NPC Genitals05 [Gen05]", "NPC Genitals06 [Gen06]" },
			{ "WWD 2", "WWD 3", "WWD 4", "WWD 5", "WWD 6", "WWD 7", "WWD 8", "WWD 9" },
			{ "VLDick02", "VLDick03", "VLDick04", "VLDick05", "VLDick06" },
			{ "BearD 4", "BearD 5", "BearD 6", "BearD 7", "BearD 8", "BearD 9" },
			{ "BoarDick01", "BoarDick02", "BoarDick03", "BoarDick04", "BoarDick05", "BoarDick06" },
			{ "CDmiddle 3", "CDPenis 1", "CDPenis 2", "CDPenis 3", "CDPenis 4", "CDPenis 5", "CDPenis 6", "CDPenis 7" },
			{ "CO 2", "CO 3", "CO 4", "CO 5", "CO 6", "CO 7", "CO 8", "CO 9" },
			{ "ElkD03", "ElkD04", "ElkD05", "ElkD06" },
			{ "DD 2", "DD 3", "DD 4", "DD 5", "DD 6" },
			{ "FD 2", "FD 3", "FD 4", "FD 5", "FD 6", "FD 7" },
			{ "GD 2", "GD 3", "GD 4", "GD 5", "GD 6", "GD 7" },
			{ "GS 2", "GS 3", "GS 4", "GS 5", "GS 6", "GS 7" },
			{ "Goat_Penis02", "Goat_Penis03", "Goat_Penis04", "Goat_Penis05", "Goat_Penis06" },
			{ "Horker_Penis02", "Horker_Penis03", "Horker_Penis04", "Horker_Penis05", "Horker_Penis06", "Horker_Penis07", "Horker_Penis08", "Horker_Penis09", "Horker_Penis10" },
			{ "HS 3", "HS 4", "HS 5", "HS 6", "HS 7" },
			{ "Mammoth Gen05", "Mammoth Gen06", "Mammoth Gen07", "Mammoth Gen08", "Mammoth Gen09", "Mammoth Gen10" },
			{ "RD Base", "RD 2", "RD 3", "RD 4", "RD 5" },
			{ "SCD 2", "SCD 3", "SCD 4", "SCD 5", "SCD 6", "SCD 7" },
			{ "SkeeverD 01", "SkeeverD 02", "SkeeverD 03", "SkeeverD 04", "SkeeverD 05", "SkeeverD 06", "SkeeverD 07" },
			{ "TD 2", "TD 3", "TD 4", "TD 5", "TD 6", "TD 7" }
		};
		return c;
	}

	// 0.4.0: a skeleton DeepFit does not know: its schlong bones guessed from their names ("...Penis..", "..Dick..",
	// "..Cock..", "Genital..", "Schlong..", "..Shaft..", "Knot..", or the short "XD 2" style many rigs use)
	bool guessGenital(std::string_view name)
	{
		std::string n(name);
		for (auto& c : n)
			c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
		for (const char* bad : { "scrot", "testic", "ball", "head", "hand", "pad", "shield", "beard", "braid", "sheath", "weapon", "magic", "camera" })
			if (n.find(bad) != std::string::npos)
				return false;
		for (const char* kw : { "penis", "dick", "cock", "genital", "schlong", "shaft", "phallus", "glans", "knot" })
			if (n.find(kw) != std::string::npos)
				return true;
		// "td 2", "scd 3", "skeeverd 01", "elkd03": letters ending in d, then a number
		std::size_t i = n.size();
		while (i > 0 && std::isdigit(static_cast<unsigned char>(n[i - 1])))
			--i;
		if (i == n.size() || n.size() - i > 2)
			return false;
		if (i > 0 && (n[i - 1] == ' ' || n[i - 1] == '_'))
			--i;
		if (i < 2 || n[i - 1] != 'd')
			return false;
		for (std::size_t k = 0; k + 1 < i; ++k)
			if (!std::isalpha(static_cast<unsigned char>(n[k])))
				return false;
		return i <= 9;
	}

	std::vector<NiPointer<NiAVObject>> guessChain(NiAVObject* root)
	{
		std::vector<NiPointer<NiAVObject>> best;
		std::function<void(NiAVObject*, bool)> walk = [&](NiAVObject* o, bool parentHit) {
			if (!o)
				return;
			const bool hit = !o->AsGeometry() && guessGenital(o->name.c_str() ? o->name.c_str() : "");
			if (hit && !parentHit) {   // the base of a chain: follow it down through children that are hits too
				std::vector<NiPointer<NiAVObject>> c;
				NiAVObject* cur = o;
				while (cur) {
					c.emplace_back(cur);
					NiAVObject* next = nullptr;
					if (auto* nd = cur->AsNode())
						for (auto& ch : nd->GetChildren())
							if (ch && !ch->AsGeometry() && guessGenital(ch->name.c_str() ? ch->name.c_str() : "")) { next = ch.get(); break; }
					cur = next;
				}
				if (c.size() > best.size())
					best = std::move(c);
			}
			if (auto* nd = o->AsNode())
				for (auto& ch : nd->GetChildren())
					if (ch)
						walk(ch.get(), hit);
		};
		walk(root, false);
		if (best.size() < 3)
			best.clear();
		return best;
	}

	std::vector<NiPointer<NiAVObject>> findChain(NiAVObject* root)
	{
		static const auto& kChains = schlongChains();
		std::vector<NiPointer<NiAVObject>> best;
		for (auto& names : kChains) {   // the beast chains win over the human bones a beast skeleton also carries (unused)
			std::vector<NiPointer<NiAVObject>> c;
			for (auto* n : names)
				if (auto* o = root->GetObjectByName(n))
					c.emplace_back(o);
			if (c.size() >= 3 && (best.empty() || names[0][0] != 'N'))
				best = std::move(c);
		}
		if (best.empty() && Config::get().guessRigs)   // 0.4.0: a skeleton it does not know
			best = guessChain(root);
		return best;
	}

	std::atomic<bool>                    g_capture{ false };   // 0.3.1: the menu waits for the Mega mouth key
	int                                  g_mods = 0;           // Shift 1 / Ctrl 2 / Alt 4 held now
	std::unordered_map<FormID, Receiver> g_receivers;
	std::mt19937                         g_rng{ std::random_device{}() };   // 0.4.4 (extra push, sounds)
	double                               g_clock = 0.0;   // 0.4.0: seconds since the game started (for gulps / gags)
	DeepFit::Preview                     g_preview;        // 0.4.0: the menu's test page
	std::unordered_map<FormID, Partner>  g_partners;
	// 0.4.5: his schlong's bones as they were at the start of this frame - last frame's final pose, after PPA bent it into
	// the mouth (PPA does that in its own late hook, after DeepFit had read the animation's pose: a canine aimed into his
	// mouth on screen was read 25 units off and the mouth never opened)
	std::unordered_map<const NiAVObject*, NiTransform> g_snap;
	bool                                               g_useSnap = false;
	std::unordered_map<const NiTransform*, NiTransform> g_snapT;   // 0.4.7: the same, by the transform the game draws with
	const NiTransform& tOf(const NiTransform* t)
	{
		if (g_useSnap) {
			auto it = g_snapT.find(t);
			if (it != g_snapT.end())
				return it->second;
		}
		return *t;
	}
	const NiTransform& wOf(const NiAVObject* o)
	{
		if (g_useSnap) {
			auto it = g_snap.find(o);
			if (it != g_snap.end())
				return it->second;
		}
		return o->world;
	}
	// a partner's bone as the game draws it (its own transform, or the one its mesh is drawn with), as the last frame ended
	const NiTransform& wP(const Partner& p, const NiAVObject* o)
	{
		auto it = p.drawnBy.find(o);
		return it != p.drawnBy.end() ? tOf(it->second) : wOf(o);
	}
	float                                g_logTimer = 0.0f;

	// ------------------------------------------------------------------ small helpers

	inline float dot(const NiPoint3& a, const NiPoint3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
	inline float len(const NiPoint3& a) { return std::sqrt(dot(a, a)); }
	inline NiPoint3 norm(const NiPoint3& a)
	{
		float l = len(a);
		return l > 1e-6f ? NiPoint3(a.x / l, a.y / l, a.z / l) : NiPoint3(0, 0, 0);
	}

	float half2float(std::uint16_t h)
	{
		std::uint32_t s = (h >> 15) & 1, e = (h >> 10) & 0x1f, m = h & 0x3ff;
		float         v;
		if (e == 0)
			v = std::ldexp(static_cast<float>(m), -24);
		else if (e == 31)
			v = m ? NAN : INFINITY;
		else
			v = std::ldexp(static_cast<float>(m | 0x400), static_cast<int>(e) - 25);
		return s ? -v : v;
	}

	bool endsWith(const std::string_view s, const std::string_view suf)
	{
		return s.size() >= suf.size() && s.compare(s.size() - suf.size(), suf.size(), suf) == 0;
	}

	// "Upper Lip Center" or "<prefix> Upper Lip Center", but never "... Pre Upper Lip Center"
	int matchSpec(const char* name, const std::vector<HeadSpec>& specs)
	{
		if (!name)
			return -1;
		std::string_view n(name);
		for (int i = 0; i < static_cast<int>(specs.size()); ++i) {
			std::string_view suf(specs[i].name);
			if (n == suf)
				return i;
			if (n.size() > suf.size() && endsWith(n, suf) && n[n.size() - suf.size() - 1] == ' ') {
				std::string_view before = n.substr(0, n.size() - suf.size() - 1);
				if (!endsWith(before, "Pre"))
					return i;
			}
		}
		return -1;
	}

	bool knownSchlongBone(std::string_view n)   // 0.4.0: one of the rigs DeepFit knows
	{
		if (n.find("NPC Genitals0") != std::string_view::npos)
			return true;
		static const std::unordered_set<std::string_view> kAll = [] {
			std::unordered_set<std::string_view> s;
			for (auto& ch : schlongChains())
				for (auto* b : ch)
					s.insert(b);
			return s;
		}();
		return kAll.contains(n);
	}

	bool isGenitalBone(const char* name)
	{
		if (!name)
			return false;
		std::string_view n(name);
		// "NPC Genitals01 [Gen01]" ... "NPC Genitals06 [Gen06]"; 0.3.1: a werewolf's (WWD 2-9), a Vampire Lord's (VLDick02-06)
		// and a werebear's (BearD 4-9) hang on their own bones
		if (n.find("Scrotum") != std::string_view::npos)
			return false;
		return knownSchlongBone(n) || (Config::get().guessRigs && guessGenital(n));   // 0.4.0: guessed by name
	}

	template <class F>
	void traverse(NiAVObject* obj, F&& f)
	{
		if (!obj)
			return;
		f(obj);
		if (auto node = obj->AsNode()) {
			for (auto& child : node->GetChildren()) {
				if (child)
					traverse(child.get(), f);
			}
		}
	}

	// ------------------------------------------------------------------ receiver cache

	// 0.4.9: every bone a mesh of his is skinned to now. A rebuilt face can leave old copies of his lip bones hung on his
	// skeleton - they still move with his head, but his face is no longer drawn with them: a canine scene read his mouth
	// ~24 units off his lips (under his chin) until "Refresh my head", and the mouth never opened
	void skinnedBones(NiAVObject* root, std::unordered_set<const NiAVObject*>& out)
	{
		traverse(root, [&](NiAVObject* o) {
			auto* g = o ? o->AsGeometry() : nullptr;
			if (!g)
				return;
			auto* si = g->GetGeometryRuntimeData().skinInstance.get();
			if (!si || !si->bones || !si->skinData)
				return;
			const std::uint32_t nb = si->skinData->bones;
			for (std::uint32_t i = 0; i < nb; ++i)
				if (si->bones[i])
					out.insert(si->bones[i]);
		});
	}

	Receiver* getReceiver(Actor* actor, float delta)
	{
		auto* root = actor->Get3D(false);
		if (!root)
			return nullptr;
		auto& r = g_receivers[actor->GetFormID()];
		if (r.root == root && r.found > 0 && r.headsGen == Heads::Generation()) {   // 0.4.6: head files reloaded = look again
			// 0.3.2: the face can be rebuilt under the same body (a face paint / overlay change, an expression mod): the lip
			// bones we kept are then the old, detached ones - frozen where the face once was, ~500 units off in the log -
			// so the mouth never met the shaft. Still on his head, near it? Then keep them, else look them up again.
			auto& up = r.bones[kUpperCenter];
			const bool live = r.head && r.head->parent && up.valid && up.node && up.node->parent && up.node->parent == up.parent.get() &&
			                  len(up.node->world.translate - r.head->world.translate) < 40.0f;
			bool stale = false;
			if (live && g_clock - r.skinCheckAt > 0.5) {   // 0.4.9: still the lip bones his face is drawn with?
				r.skinCheckAt = g_clock;
				std::unordered_set<const NiAVObject*> skin;
				skinnedBones(root, skin);
				bool anyLip = false;
				for (auto* b : skin)
					if (b && matchSpec(b->name.c_str(), r.specs) == kUpperCenter) {
						anyLip = true;
						break;
					}
				stale = anyLip && !skin.contains(up.node.get());
			}
			if (live && !stale)
				return &r;
			if (Config::get().debugLog && stale)
				logs::info("DeepFit: {:08X}'s face is drawn with other lip bones than DeepFit had - finding them again", actor->GetFormID());
			else if (Config::get().debugLog)
				logs::info("DeepFit: {:08X}'s face was rebuilt - finding the mouth bones again", actor->GetFormID());
			r.found = 0;
			r.checked = false;
		}
		if (r.root == root && r.found == 0 && r.checked && r.headsGen == Heads::Generation()) {
			// the facegen head attaches a little after the body 3D: keep looking every 2 s
			r.retry -= delta;
			if (r.retry > 0.0f)
				return nullptr;
		}

		// (re)build - 0.4.7: his face rebuilt under the same body (often in a scene) keeps where his mouth was, instead of
		// snapping shut and easing open again (the chin jittered every few seconds)
		float keepMorph[kMorphs];
		std::copy(std::begin(r.morph), std::end(r.morph), keepMorph);
		const bool keep = r.root == root;
		r = Receiver{};
		if (keep)
			std::copy(std::begin(keepMorph), std::end(keepMorph), r.morph);
		r.root = root;
		r.actor = actor->GetHandle();
		r.checked = true;
		r.retry = 2.0f;
		auto headObj = root->GetObjectByName("NPC Head [Head]");
		if (!headObj) {
			if (Config::get().debugLog && actor->IsPlayerRef())
				logs::info("DeepFit: player 3D has no 'NPC Head [Head]' node yet");
			return nullptr;
		}
		r.head = NiPointer<NiAVObject>(headObj);
		if (auto* mg = root->GetObjectByName("NPC Head MagicNode [Hmag]"))
			r.hmag = NiPointer<NiAVObject>(mg);
		// 0.4.6: his head profile (by his face mesh, then his race) and the bones it has
		r.headsGen = Heads::Generation();
		{
			std::string race, mesh;
			bool        fem = false;
			if (auto* rc = actor->GetRace(); rc && rc->GetFormEditorID())
				race = rc->GetFormEditorID();
			if (auto* base = actor->GetActorBase()) {
				fem = base->GetSex() == SEX::kFemale;
				if (auto* hp = base->GetCurrentHeadPartByType(BGSHeadPart::HeadPartType::kFace); hp && hp->GetModel())
					mesh = hp->GetModel();
			}
			r.profile = Heads::Find(race, mesh, fem);
			r.specs = specsFor(r.profile);
			static std::unordered_set<FormID> told;   // once per actor, not on every 2 s retry of a head without the bones
			if ((r.profile >= 0 || Config::get().debugLog) && told.insert(actor->GetFormID()).second)
				logs::info("DeepFit: {:08X} race {} head {} - {}", actor->GetFormID(), race, mesh,
					r.profile >= 0 ? "head profile \"" + Heads::All()[r.profile].name + "\"" : std::string("DeepFit's own bones"));
		}
		std::unordered_set<const NiAVObject*> skin;   // 0.4.9: a copy his face is skinned to beats one it is not
		skinnedBones(root, skin);
		traverse(root, [&](NiAVObject* o) {
			int i = matchSpec(o->name.c_str(), r.specs);
			if (i < 0 || !o->parent)
				return;
			auto& b = r.bones[i];
			if (b.valid) {   // first match wins - unless only this one is drawn with
				if (skin.contains(b.node.get()) || !skin.contains(o))
					return;
			} else
				++r.found;
			b.node = NiPointer<NiAVObject>(o);
			b.parent = NiPointer<NiNode>(o->parent);
			b.restLocal = o->local.translate;
			b.cur = 0.0f;
			b.valid = true;
		});
		// 0.5.5: no lip bones, but a head profile that opens it with a skeleton jaw bone (a werewolf's): DeepFit's own two
		// markers where its lips are, so the shaft is measured at its mouth, and the jaw bone turned open
		if ((!r.bones[kUpperCenter].valid || !r.bones[kBottomCenter].valid) && r.profile >= 0) {
			const auto& hp = Heads::All()[r.profile];
			auto*       headNode = r.head->AsNode();
			auto*       jaw = hp.jawBone.empty() ? nullptr : root->GetObjectByName(hp.jawBone.c_str());
			if (hp.hasLips && headNode && jaw) {
				const float* at[2] = { hp.upperLipAt, hp.lowerLipAt };
				const int    idx[2] = { kUpperCenter, kBottomCenter };
				for (int k = 0; k < 2; ++k) {
					const char* nm = k ? "DF Lower Lip Marker" : "DF Upper Lip Marker";
					NiPointer<NiNode> n;
					for (auto& ch : headNode->GetChildren())   // (again after a rebuild: the one already there)
						if (ch && ch->name.c_str() && std::strcmp(ch->name.c_str(), nm) == 0 && ch->AsNode())
							n = NiPointer<NiNode>(ch->AsNode());
					if (!n) {
						n = NiPointer<NiNode>(NiNode::Create(0));
						n->name = nm;
						headNode->AttachChild(n.get(), true);
					}
					n->local = NiTransform();
					n->local.translate = NiPoint3(at[k][0], at[k][1], at[k][2]);
					n->world = r.head->world * n->local;
					r.virtLips[k] = n;
					auto& b = r.bones[idx[k]];
					b.node = NiPointer<NiAVObject>(n.get());
					b.parent = NiPointer<NiNode>(headNode);
					b.restLocal = n->local.translate;
					b.cur = 0.0f;
					b.valid = true;
				}
				r.found = std::max(r.found, 2);
				r.jawBone = NiPointer<NiAVObject>(jaw);
				r.jawBoneDeg = hp.jawBoneDeg;
				r.jawSetValid = false;
				r.jawSign = 0.0f;
				logs::info("DeepFit: receiver {:08X} opens with its jaw bone ({}) - no lip bones", actor->GetFormID(), hp.jawBone);
				return &r;
			}
		}
		if (r.found == 0 || !r.bones[kUpperCenter].valid || !r.bones[kBottomCenter].valid) {
			if (Config::get().debugLog && actor->IsPlayerRef())
				logs::info("DeepFit: player head found but only {} mouth bones matched (need Upper/Bottom Lip Center)", r.found);
			r.found = 0;
			return nullptr;
		}
		logs::info("DeepFit: receiver {:08X} has {} mouth bones", actor->GetFormID(), r.found);
		return &r;
	}

	// ------------------------------------------------------------------ partner (shaft) cache

	void buildPartner(Partner& p, NiAVObject* root)
	{
		p.shapes.clear();
		p.looked = p.genital = 0;
		p.root = root;
		p.age = 0.0f;
		p.drawnBy.clear();
		p.drawnOther = 0;
		p.chain = findChain(root);
		p.anyMesh = false;
		if (p.chain.empty() && Config::get().anyMeshAtLips) {   // 0.4.0: no schlong bones known or guessed: is any of his mesh weighted to one?
			bool named = false;
			traverse(root, [&](NiAVObject* o) { if (!named && o && !o->AsGeometry() && isGenitalBone(o->name.c_str())) named = true; });
			p.anyMesh = !named;
		}
		traverse(root, [&](NiAVObject* o) {
			auto* geom = o->AsGeometry();
			if (!geom)
				return;
			auto& rd = geom->GetGeometryRuntimeData();
			auto* si = rd.skinInstance.get();
			if (!si)
				return;
			auto* sd = si->skinData.get();
			auto* sp = si->skinPartition.get();
			if (!sd || !sp || sp->numPartitions == 0 || !si->bones || !sd->boneData)
				return;
			const std::uint32_t nb = sd->bones;
			std::vector<bool>   isGen(nb, false);
			bool                any = false;
			for (std::uint32_t i = 0; i < nb; ++i) {
				auto* b = si->bones[i];
				if (b && isGenitalBone(b->name.c_str())) {
					isGen[i] = true;
					any = true;
				}
			}
			++p.looked;
			// 0.4.5: not the shaft: strings of cum (OStrands' "Strand"), drips, the balls / sheath and the like are weighted to
			// the schlong's bones too - read as the shaft they sat above his lips, made it 11 units wide at them, and the mouth
			// lost track of it
			if (any) {
				std::string gn = geom->name.c_str() ? geom->name.c_str() : "";
				for (auto& c : gn)
					c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
				for (const char* bad : { "strand", "cum", "drip", "drool", "saliva", "fluid", "glint", "sheen", "ball", "scrot", "testic", "sack", "sheath" })
					if (gn.find(bad) != std::string::npos) {
						any = false;
						if (Config::get().debugLog && !p.told)
							logs::info("DeepFit: {} is not the shaft - not read", geom->name.c_str() ? geom->name.c_str() : "?");
						break;
					}
			}
			// 0.4.2: a hidden part is not his schlong as it is drawn - at the start of a scene a hidden copy (the soft one, or
			// one about to be swapped out) was read and his schlong was taken to be 17 units below where it showed, so the
			// mouth never opened until the scene was started again
			if (any) {
				for (NiAVObject* up = geom; up && up != root->parent; up = up->parent) {
					if (up->GetAppCulled()) {
						any = false;
						if (Config::get().debugLog)
							logs::info("DeepFit: {} is hidden - not read as his schlong", geom->name.c_str() ? geom->name.c_str() : "?");
						break;
					}
				}
			}
			if (!any && p.anyMesh) {   // 0.4.0: nothing recognisable at all - every part of him counts (DeepFit keeps what is at his lips)
				if (sp->vertexCount > 20000)
					return;
				std::fill(isGen.begin(), isGen.end(), true);
				any = true;
			}
			if (!any)
				return;
			++p.genital;
			auto& part = sp->partitions[0];
			auto  flags = part.vertexDesc.GetFlags();
			if (!(flags & BSGraphics::Vertex::VF_SKINNED) || !(flags & BSGraphics::Vertex::VF_VERTEX))
				return;
			if (!part.buffData || !part.buffData->rawVertexData)
				return;
			const std::uint32_t vSize = part.vertexDesc.GetSize();
			const std::uint32_t skinOff = part.vertexDesc.GetAttributeOffset(BSGraphics::Vertex::VA_SKINNING);
			const std::uint8_t* data = part.buffData->rawVertexData;
			// 0.1.4: full or half precision positions: the size of the position block before the next attribute (as OStrands reads it)
			std::uint32_t posBytes = vSize;
			for (std::uint8_t at = BSGraphics::Vertex::VA_TEXCOORD0; at < BSGraphics::Vertex::VA_COUNT; ++at) {
				if (part.vertexDesc.HasFlag(static_cast<BSGraphics::Vertex::Flags>(1u << at)))
					posBytes = std::min(posBytes, part.vertexDesc.GetAttributeOffset(static_cast<BSGraphics::Vertex::Attribute>(at)));
			}
			const bool fullPrec = posBytes >= 12;

			// 0.4.4: which partition each vertex is drawn in. A mesh in several partitions (many non-human schlongs) may number
			// its vertices' bones within their partition, not across the whole mesh: read that way the points landed on the
			// wrong bones, 20-25 units from where the schlong was drawn, and the mouth never opened. Both readings are tried
			// and the one whose points sit on their own bones is kept.
			std::vector<std::int16_t> partOf;
			if (sp->numPartitions > 1) {
				partOf.assign(sp->vertexCount, -1);
				for (std::uint32_t pi = 0; pi < sp->numPartitions; ++pi) {
					auto&                pp = sp->partitions[pi];
					const std::uint16_t* idx = (pp.buffData && pp.buffData->rawIndexData) ? pp.buffData->rawIndexData : pp.triList;
					if (!idx)
						continue;
					for (std::uint32_t t = 0; t < static_cast<std::uint32_t>(pp.triangles) * 3u; ++t)
						if (idx[t] < sp->vertexCount && partOf[idx[t]] < 0)
							partOf[idx[t]] = static_cast<std::int16_t>(pi);
				}
			}
			auto readVerts = [&](bool local) {
				ShaftShape out;
				out.geom = NiPointer<BSGeometry>(geom);
				for (std::uint32_t j = 0; j < sp->vertexCount; ++j) {
					const std::uint8_t*  v = data + static_cast<std::size_t>(j) * vSize;
					const std::uint16_t* hw = reinterpret_cast<const std::uint16_t*>(v + skinOff);
					const std::uint8_t*  bi0 = reinterpret_cast<const std::uint8_t*>(hw + 4);
					std::uint32_t        bi[4];
					for (int k = 0; k < 4; ++k) {
						bi[k] = bi0[k];
						if (local && !partOf.empty() && partOf[j] >= 0) {
							auto& pp = sp->partitions[partOf[j]];
							bi[k] = (pp.bones && bi0[k] < pp.numBones) ? pp.bones[bi0[k]] : 0xFFFF;
						}
					}
					float w[4];
					float genW = 0.0f, tot = 0.0f;
					for (int k = 0; k < 4; ++k) {
						w[k] = half2float(hw[k]);
						if (!(w[k] > 0.0f) || bi[k] >= nb || !si->bones[bi[k]])
							w[k] = 0.0f;
						tot += w[k];
						if (w[k] > 0.0f && isGen[bi[k]])
							genW += w[k];
					}
					if (tot <= 0.0f || genW / tot < 0.6f)
						continue;
					ShaftVert sv;
					if (fullPrec)
						sv.skinPos = *reinterpret_cast<const NiPoint3*>(v);
					else {   // 0.1.4: half-precision positions (most SE meshes) - 0.1.3 read them as floats and got scattered points
						const std::uint16_t* h3 = reinterpret_cast<const std::uint16_t*>(v);
						sv.skinPos = NiPoint3(half2float(h3[0]), half2float(h3[1]), half2float(h3[2]));
					}
					for (int k = 0; k < 4; ++k) {
						sv.bone[k] = static_cast<std::uint8_t>(std::min<std::uint32_t>(bi[k], 255u));
						sv.weight[k] = (bi[k] < 256u) ? w[k] / tot : 0.0f;
					}
					out.verts.push_back(sv);
				}
				return out;
			};
			// how far the points land from the bone that moves each most (small = read right)
			auto offBones = [&](const ShaftShape& sh) {
				std::vector<float> dd;
				const std::size_t  step = std::max<std::size_t>(1, sh.verts.size() / 80);
				for (std::size_t i = 0; i < sh.verts.size(); i += step) {
					const auto& sv = sh.verts[i];
					int         kb = 0;
					for (int k = 1; k < 4; ++k)
						if (sv.weight[k] > sv.weight[kb]) kb = k;
					const std::uint32_t b = sv.bone[kb];
					if (b >= nb || !si->bones[b])
						continue;
					const NiTransform& bw = (si->boneWorldTransforms && si->boneWorldTransforms[b]) ? tOf(si->boneWorldTransforms[b]) : wOf(si->bones[b]);
					const NiPoint3 q = (bw * sd->boneData[b].skinToBone) * sv.skinPos;
					dd.push_back(len(q - bw.translate));
				}
				if (dd.empty())
					return 1e9f;
				std::nth_element(dd.begin(), dd.begin() + dd.size() / 2, dd.end());
				return dd[dd.size() / 2];
			};
			ShaftShape shape = readVerts(false);
			if (!partOf.empty()) {
				ShaftShape alt = readVerts(true);
				const float a = offBones(shape), b = offBones(alt);
				if (!alt.verts.empty() && (shape.verts.empty() || b < a * 0.7f)) {
					if (Config::get().debugLog)
						logs::info("DeepFit: {} numbers its bones per partition ({} partitions; {:.1f} from its bones read that way, {:.1f} the other)",
							geom->name.c_str() ? geom->name.c_str() : "?", sp->numPartitions, b, a);
					shape = std::move(alt);
				}
			}
			if (!shape.verts.empty() && si->boneWorldTransforms) {   // 0.4.7: bones drawn with another bone's transform
				int other = 0;
				for (std::uint32_t i = 0; i < nb; ++i)
					if (si->bones[i] && si->boneWorldTransforms[i] && si->boneWorldTransforms[i] != &si->bones[i]->world) {
						p.drawnBy[si->bones[i]] = si->boneWorldTransforms[i];
						++other;
					}
				p.drawnOther += other;
				if (other && Config::get().debugLog)
					logs::info("DeepFit: {} is drawn with other bones than it names ({} of {}) - read as it is drawn",
						geom->name.c_str() ? geom->name.c_str() : "?", other, nb);
			}
			if (!shape.verts.empty()) {
				if (Config::get().debugLog)   // 0.4.2: which parts of him are read
					logs::info("DeepFit: reading {} ({} of its {} vertices)", geom->name.c_str() ? geom->name.c_str() : "?", shape.verts.size(), sp->vertexCount);
				p.shapes.push_back(std::move(shape));
			}
		});
		// 0.3.1: his thickness, from the mesh itself (skin space: no bones needed): the spread of the shaft vertices round
		// the line through them, the outer part (a few inner ones do not shrink it)
		std::vector<NiPoint3> all;
		for (auto& sh : p.shapes)
			for (auto& v : sh.verts)
				all.push_back(v.skinPos);
		const bool beast = !p.chain.empty() && p.chain[0]->name.c_str()[0] != 'N';
		p.radius = beast ? 2.5f : 1.6f;
		if (all.size() >= 30) {
			NiPoint3 c(0, 0, 0);
			for (auto& v : all) c += v;
			c = c * (1.0f / static_cast<float>(all.size()));
			NiPoint3 a = all[0], b = all[0];   // its length: the two vertices farthest apart (two passes)
			float    d = -1.0f;
			for (auto& v : all) if (len(v - c) > d) { d = len(v - c); a = v; }
			d = -1.0f;
			for (auto& v : all) if (len(v - a) > d) { d = len(v - a); b = v; }
			const NiPoint3 ax = norm(b - a);
			std::vector<float> r;
			for (auto& v : all) {
				NiPoint3 q = v - c;
				q = q - ax * dot(q, ax);
				r.push_back(len(q));
			}
			std::sort(r.begin(), r.end());
			const float m = r[r.size() * 7 / 10];
			if (m > 0.4f && m < 8.0f)
				p.radius = m;
		}
	}

	// 0.3.1: the shaft as a tube round his bone line (base to tip, the tip a little past the last bone)
	void tubePoints(const Partner& p, std::vector<NiPoint3>& out)
	{
		std::vector<NiPoint3> b;
		for (auto& n : p.chain)
			if (n) b.push_back(wP(p, n.get()).translate);
		if (b.size() < 2)
			return;
		const NiPoint3 last = b.back() - b[b.size() - 2];
		b.push_back(b.back() + norm(last) * std::min(1.5f, len(last) + 0.5f));
		for (std::size_t i = 0; i + 1 < b.size(); ++i) {
			const NiPoint3 d = b[i + 1] - b[i];
			const float    L = len(d);
			if (L < 1e-3f)
				continue;
			const NiPoint3 ax = d * (1.0f / L);
			NiPoint3       u = std::fabs(ax.z) < 0.9f ? NiPoint3(0, 0, 1) : NiPoint3(1, 0, 0);
			u = norm(u - ax * dot(u, ax));
			const NiPoint3 v(ax.y * u.z - ax.z * u.y, ax.z * u.x - ax.x * u.z, ax.x * u.y - ax.y * u.x);
			const int      steps = std::max(1, static_cast<int>(L / 0.4f));
			for (int s = 0; s < steps; ++s) {
				const float    t = static_cast<float>(s) / static_cast<float>(steps);
				const bool     tip = i + 2 == b.size();
				const float    r = p.radius * (tip ? 1.0f - 0.5f * t : 1.0f);
				const NiPoint3 c = b[i] + d * t;
				for (int k = 0; k < 12; ++k) {
					const float a = 0.5235988f * static_cast<float>(k);
					out.push_back(c + u * (std::cos(a) * r) + v * (std::sin(a) * r));
				}
			}
		}
	}

	Partner* getPartner(Actor* actor, float delta)
	{
		auto* root = actor->Get3D(false);
		if (!root)
			return nullptr;
		auto& p = g_partners[actor->GetFormID()];
		p.age += delta;
		if (p.root != root || p.age > Config::get().cacheRefreshSeconds) {
			std::size_t before = p.shapes.size();
			buildPartner(p, root);
			// 0.4.0: how he was read (the menu shows it)
			if (p.anyMesh)
				p.source = "anything at his lips (no schlong bones found)";
			else if (!p.chain.empty() && !knownSchlongBone(p.chain[0]->name.c_str() ? p.chain[0]->name.c_str() : "")) {
				p.source = "guessed bones:";
				for (std::size_t i = 0; i < p.chain.size() && i < 6; ++i)
					p.source += std::string(i ? ", " : " ") + (p.chain[i]->name.c_str() ? p.chain[i]->name.c_str() : "?");
				if (p.chain.size() > 6)
					p.source += ", ...";
			} else if (!p.chain.empty())
				p.source = std::string("known skeleton (") + (p.chain[0]->name.c_str() ? p.chain[0]->name.c_str() : "?") + ")";
			else
				p.source = "his mesh (schlong bones found by name)";
			if (Config::get().debugLog && (p.shapes.size() != before || (p.shapes.empty() && !p.told))) {
				p.told = true;
				logs::info("DeepFit: partner {:08X} ({}) shaft shapes {} ({} skinned shapes looked at, {} weighted to NPC Genitals01-06)",
					actor->GetFormID(), actor->GetDisplayFullName(), p.shapes.size(), p.looked, p.genital);
			}
		}
		return (p.shapes.empty() && p.chain.size() < 2) ? nullptr : &p;
	}

	void skinPartner(Partner& p)
	{
		if (p.skinnedThisFrame)
			return;
		p.skinnedThisFrame = true;
		p.world.clear();
		for (auto& shape : p.shapes) {
			auto* geom = shape.geom.get();
			auto& rd = geom->GetGeometryRuntimeData();
			auto* si = rd.skinInstance.get();
			if (!si || !si->skinData || !si->bones)
				continue;
			auto*               sd = si->skinData.get();
			const std::uint32_t nb = sd->bones;
			// per-bone skin->world transform
			std::vector<NiTransform> xf(nb);
			std::vector<bool>        ok(nb, false);
			for (std::uint32_t i = 0; i < nb; ++i) {
				auto* b = si->bones[i];
				if (!b)
					continue;
				const NiTransform* dt = si->boneWorldTransforms ? si->boneWorldTransforms[i] : nullptr;   // 0.4.7: as the game draws it
				xf[i] = (dt ? tOf(dt) : wOf(b)) * sd->boneData[i].skinToBone;
				ok[i] = true;
			}
			for (auto& sv : shape.verts) {
				NiPoint3 acc(0, 0, 0);
				float    wt = 0.0f;
				for (int k = 0; k < 4; ++k) {
					if (sv.weight[k] <= 0.0f || sv.bone[k] >= nb || !ok[sv.bone[k]])
						continue;
					NiPoint3 q = xf[sv.bone[k]] * sv.skinPos;
					acc.x += q.x * sv.weight[k];
					acc.y += q.y * sv.weight[k];
					acc.z += q.z * sv.weight[k];
					wt += sv.weight[k];
				}
				if (wt > 0.0f)
					p.world.emplace_back(acc.x / wt, acc.y / wt, acc.z / wt);
			}
		}
		// 0.3.1: are the skinned points where his schlong is? (median distance to his bone line)
		if (p.chain.size() >= 2) {
			std::vector<float> dd;
			const std::size_t step = std::max<std::size_t>(1, p.world.size() / 60);
			for (std::size_t i = 0; i < p.world.size(); i += step) {
				float best = 1e9f;
				for (std::size_t k = 0; k + 1 < p.chain.size(); ++k) {
					const NiPoint3 a = wP(p, p.chain[k].get()).translate, b = wP(p, p.chain[k + 1].get()).translate, ab = b - a;
					const float    l2 = dot(ab, ab);
					const float    t = l2 > 1e-6f ? std::clamp(dot(p.world[i] - a, ab) / l2, 0.0f, 1.0f) : 0.0f;
					best = std::min(best, len(p.world[i] - (a + ab * t)));
				}
				dd.push_back(best);
			}
			float off = 1e9f;
			if (!dd.empty()) {
				std::nth_element(dd.begin(), dd.begin() + dd.size() / 2, dd.end());
				off = dd[dd.size() / 2];
			}
			const int method = off < p.radius * 2.0f + 3.0f ? 0 : 1;
			if (method == 1) {
				p.world.clear();
				tubePoints(p, p.world);
			}
			if (method != p.method && Config::get().debugLog)
				logs::info("DeepFit: shaft from {} ({} points, thickness {:.1f} across; the skinned mesh sits {:.1f} from his bones){}",
					method == 0 ? "his mesh" : "his bone line", p.world.size(), p.radius * 2.0f, off < 1e8f ? off : -1.0f,
					method == 1 ? " - the mesh reads wrong here, the bones are used" : "");
			p.method = method;
			p.meshOff = off;
		}
	}

	// ------------------------------------------------------------------ per receiver solve

	// 0.4.1: the test schlong's thickness along it (1 = as set), sIn units past his lips, its tip D deep
	float previewProfile(float sIn, float D, int shape)
	{
		const auto& pv = g_preview;
		float k = 1.0f;
		if (shape == 1) {   // horse: a medial ring, and the flared head
			k += 0.3f * std::exp(-std::pow((sIn - D * 0.45f) / 0.35f, 2.0f));
			if (sIn > D - 2.2f) k = std::max(k, 1.45f);
		} else if (shape == 2)   // knot: a bulb near the base
			k += 0.85f * std::exp(-std::pow((sIn - std::min(2.0f, D * 0.3f)) / 0.9f, 2.0f));
		else if (shape == 5) {   // a bump, BumpAt along it from where it enters (3 units out) to the tip
			const float at = -3.0f + std::clamp(pv.bumpAt, 0.0f, 1.0f) * (std::max(pv.depth, 0.5f) + 3.0f);
			k += pv.bumpSize * std::exp(-std::pow((sIn - at) / 0.9f, 2.0f));
		}
		return k;
	}
	double g_strokePhase = 0.0;

	// 0.4.1: the test schlong placed and angled in his face's frame (right, forward, up from the middle of his lips): tip up
	// by Pitch, to his right by Yaw, entering Up / Side off the middle. The lips fit what is placed, so it still collides.
	float g_previewDrop = 0.0f;   // 0.4.6: the test schlong lowered to the middle of the open mouth (half the jaw's drop)
	NiPoint3 previewPlace(const NiPoint3& p)
	{
		const auto& pv = g_preview;
		const float a = -pv.pitch * 0.01745329f, b = pv.yaw * 0.01745329f;
		NiPoint3 q(p.x, p.y * std::cos(a) - p.z * std::sin(a), p.y * std::sin(a) + p.z * std::cos(a));
		q = NiPoint3(q.x * std::cos(b) - q.y * std::sin(b), q.x * std::sin(b) + q.y * std::cos(b), q.z);
		return NiPoint3(q.x + pv.side, q.y, q.z + pv.up + g_previewDrop);
	}

	void solve(Receiver& r, const std::vector<NiPoint3>& shaft, float delta, bool preview = false)
	{
		auto& cfg = cfgOf(r);
		r.full = false;

		// rest world position of every driven bone
		NiPoint3 rest[kMaxSpecs];
		for (int i = 0; i < r.nspecs(); ++i) {
			auto& b = r.bones[i];
			if (b.valid)
				rest[i] = b.parent->world * b.restLocal;
		}
		// 0.1.3: his face's own axes, from the mouth itself - right from corner to corner, forward towards the head's
		// magic node (in front of his face), up from the two. The head bone's axes are not right / forward / up
		// (0.1.2 pushed only the lower lip, always to its limit, and never the upper lip or the corners).
		{
			const NiTransform& H = r.head->world;
			NiPoint3 X = norm(H.rotate * NiPoint3(1, 0, 0));
			if (r.bones[7].valid && r.bones[6].valid) {
				const NiPoint3 across = rest[7] - rest[6];   // Corner Lip Right - Corner Lip Left
				if (len(across) > 0.05f)
					X = norm(across);
			}
			NiPoint3 F = r.hmag ? r.hmag->world.translate - H.translate : H.rotate * NiPoint3(0, 1, 0);
			F = F - X * dot(F, X);
			NiPoint3 Y = len(F) > 0.01f ? norm(F) : norm(H.rotate * NiPoint3(0, 1, 0));
			NiPoint3 Z = norm(NiPoint3(X.y * Y.z - X.z * Y.y, X.z * Y.x - X.x * Y.z, X.x * Y.y - X.y * Y.x));
			if (r.bones[kUpperCenter].valid && r.bones[kBottomCenter].valid && dot(rest[kUpperCenter] - rest[kBottomCenter], Z) < 0.0f) {
				X = X * -1.0f;   // corners the other way round: keep up pointing up
				Z = Z * -1.0f;
			}
			r.fx = X;
			r.fy = Y;
			r.fz = Z;
			r.axesOk = true;
			if (cfg.debugLog && !r.toldFrame) {
				r.toldFrame = true;
				const NiPoint3 hx = norm(H.rotate * NiPoint3(1, 0, 0)), hy = norm(H.rotate * NiPoint3(0, 1, 0)), hz = norm(H.rotate * NiPoint3(0, 0, 1));
				logs::info("DeepFit: face axes from the mouth (right/forward/up) against the head bone's x/y/z: right {:.2f}/{:.2f}/{:.2f}, forward {:.2f}/{:.2f}/{:.2f}, up {:.2f}/{:.2f}/{:.2f}",
					dot(X, hx), dot(X, hy), dot(X, hz), dot(Y, hx), dot(Y, hy), dot(Y, hz), dot(Z, hx), dot(Z, hy), dot(Z, hz));
			}
		}
		const NiPoint3 X = r.fx, Y = r.fy, Z = r.fz;
		const NiPoint3 C = (rest[kUpperCenter] + rest[kBottomCenter]) * 0.5f;
		r.mouthC = C;

		// shaft vertices in head space, relative to the mouth centre
		std::vector<NiPoint3> hs;
		hs.reserve(shaft.size());
		for (auto& v : shaft) {
			NiPoint3 p = v - C;
			hs.emplace_back(dot(p, X), dot(p, Y), dot(p, Z));
		}
		// 0.4.0: the test page - a made-up schlong straight into his mouth (no scene needed), with the chosen shape
		if (preview) {
			{   // 0.4.6: the jaw opens downward - centre it in the opening, not at the lips' middle at rest
				const float drop = headJawDrop(r);
				const float want = g_preview.autoCentre ? -0.5f * drop * std::clamp(r.morph[0] / 100.0f, 0.0f, 1.0f) : 0.0f;
				g_previewDrop += (want - g_previewDrop) * (1.0f - std::exp(-delta * 6.0f));
			}
			hs.clear();
			const auto& pv = g_preview;
			float D = std::max(pv.depth, 0.5f);
			if (pv.strokes) {   // in and out (0.4.1: its speed and length set on the test page)
				g_strokePhase += delta * std::clamp(pv.speed, 0.05f, 4.0f) * 6.2831853;
				const float L = std::clamp(pv.strokeLen, 0.0f, 0.9f);
				D *= 1.0f - L + L * std::sin(static_cast<float>(std::fmod(g_strokePhase, 6.2831853)));
			}
			float T = std::max(pv.thick, 0.2f), W = std::max(pv.width, 0.2f);
			int   pshape = pv.shape;
			if (pv.size >= 0) {   // 0.4.0: a TNG size - a plain, round schlong (1.3 half thickness at TNG's 1.0)
				const auto ts = DeepFit::GetTngSizes();
				const int  cat = pv.size == 5 ? (ts.mine >= 0 ? ts.mine : 2) : std::clamp(pv.size, 0, 4);
				T = W = 1.3f * ts.scale[cat];
				pshape = 0;
			}
			r.previewD = D;
			r.previewR = 0.5f * (T + W);
			r.previewShape = pshape;
			// 0.5.6: the rings counted back from the tip - counted from a fixed start, the last ring (its tip) moved in
			// 0.25 steps as it slid in and out, and the jaw, tongue, lips and his eyes stepped with it (a twitch at the
			// slow ends of a stroke)
			for (float sIn = D; sIn >= -3.0f; sIn -= 0.25f) {
				const float k = previewProfile(sIn, D, pshape);
				const float tipK = std::sqrt(std::clamp((D - sIn) / 0.8f, 0.0f, 1.0f));   // the rounded tip
				const float rz = T * k * std::max(tipK, 0.15f), rx = W * k * std::max(tipK, 0.15f);
				for (int a = 0; a < 16; ++a) {
					const float ang = a * 0.3926991f;
					hs.emplace_back(previewPlace(NiPoint3(rx * std::cos(ang), -sIn, rz * std::sin(ang))));
				}
			}
			r.shape = pshape == 3 ? 6 : pshape == 4 ? 7 : 0;
			r.shapeAt = g_clock;
		}

		{   // 0.5.9: the shaft's points (world) for the throat
			r.shaftWorld.clear();
			r.shaftWorld.reserve(hs.size());
			for (auto& q : hs)
				r.shaftWorld.push_back(C + X * q.x + Y * q.y + Z * q.z);
		}
		float target[kMaxSpecs] = {};
		// 0.1.4: is the shaft in his mouth at all (near the mouth centre, at the lips' depth)? Then every lip opens at least
		// MinOpen and the corners spread at least MinCorner, even where no shaft point falls in that lip's own lane
		bool inMouth = false;
		for (auto& p : hs) {
			if (std::fabs(p.y) < cfg.lipDepthSlab * 2.0f && p.x * p.x + p.z * p.z < 9.0f) {
				inMouth = true;
				break;
			}
		}
		// 0.2.0: the mouth opens to the shaft: its height and width where it passes the lips, and how deep the tip is
		float comp[kMaxSpecs] = {};   // how far the morphs already moved each lip (the bones then only make up the rest)
		if (DeepFit::MouthNow()) {
			float zTop = -1e9f, zBot = 1e9f, xL = 1e9f, xR = -1e9f, depth = 0.0f;
			for (auto& p : hs) {
				if (p.x * p.x + p.z * p.z > 36.0f)
					continue;
				if (std::fabs(p.y) < cfg.lipDepthSlab * 1.5f) {
					zTop = std::max(zTop, p.z);
					zBot = std::min(zBot, p.z);
					xL = std::min(xL, p.x);
					xR = std::max(xR, p.x);
				}
				depth = std::max(depth, -p.y);   // into his mouth is backwards (-forward)
			}
			bool through = zTop > zBot;
			// 0.4.5: lost for a frame or two (a thin part, a gap in his mesh) - keep the mouth as it was for a moment instead
			// of letting it snap shut and open again
			if (!through && inMouth && g_clock - r.lastThroughAt < 0.35) {
				through = true;
				zTop = 0.5f * r.lastH; zBot = -0.5f * r.lastH;
				xL = -0.5f * r.lastW; xR = 0.5f * r.lastW;
			}
			float height = through ? zTop - zBot : 0.0f;
			float width = through ? xR - xL : 0.0f;
			// 0.4.1: across the shaft, not straight up and down his face - a shaft that goes in at an angle cut straight
			// across looked much taller than it is, and the mouth opened far too wide for it
			if (through && cfg.lipsAlongShaft) {
				NiPoint3 m(0, 0, 0); int n = 0;
				for (auto& p : hs)
					if (p.y < 3.0f && p.y > -6.0f && p.x * p.x + p.z * p.z < 36.0f) { m = m + p; ++n; }
				if (n >= 24) {
					m = m * (1.0f / n);
					float cxx = 0, cxy = 0, cxz = 0, cyy = 0, cyz = 0, czz = 0;
					for (auto& p : hs)
						if (p.y < 3.0f && p.y > -6.0f && p.x * p.x + p.z * p.z < 36.0f) {
							const NiPoint3 d = p - m;
							cxx += d.x * d.x; cxy += d.x * d.y; cxz += d.x * d.z; cyy += d.y * d.y; cyz += d.y * d.z; czz += d.z * d.z;
						}
					NiPoint3 v(0.0f, -1.0f, 0.0f);
					for (int it = 0; it < 12; ++it) {
						NiPoint3 w(cxx * v.x + cxy * v.y + cxz * v.z, cxy * v.x + cyy * v.y + cyz * v.z, cxz * v.x + cyz * v.y + czz * v.z);
						const float lw = len(w);
						if (lw < 1e-6f) break;
						v = w * (1.0f / lw);
					}
					if (v.y > 0.0f) v = v * -1.0f;
					if (v.y < -0.5f) {
						const NiPoint3 org = m - v * (m.y / v.y);
						NiPoint3 ux = NiPoint3(1.0f, 0.0f, 0.0f) - v * v.x;
						ux = len(ux) > 1e-4f ? norm(ux) : NiPoint3(1.0f, 0.0f, 0.0f);
						const NiPoint3 uz(v.y * ux.z - v.z * ux.y, v.z * ux.x - v.x * ux.z, v.x * ux.y - v.y * ux.x);
						float zl = 1e9f, zh = -1e9f, xl = 1e9f, xh = -1e9f;
						int   cut = 0;
						for (auto& p : hs) {
							const NiPoint3 q = p - org;
							const float    s = dot(q, v);
							if (std::fabs(s) > cfg.lipDepthSlab * 1.5f)
								continue;
							const NiPoint3 d = q - v * s;
							const float    px = dot(d, ux), pz = dot(d, uz);
							if (px * px + pz * pz > 36.0f)
								continue;
							zl = std::min(zl, pz); zh = std::max(zh, pz); xl = std::min(xl, px); xh = std::max(xh, px);
							++cut;
						}
						// 0.5.3: only a real cut across the shaft - a slanted cut that caught a few points of one ring read the
						// shaft as under 1 unit tall for a moment (deep in his mouth) and the jaw jerked shut and open again
						if (zh > zl && cut >= 8 && zh - zl > 0.55f * height) {
							height = std::min(height, zh - zl);
							width = std::min(width, xh - xl);
						}
					}
				}
			}
			// 0.5.3: and a sudden drop to under half while the shaft is still well inside his mouth is a gap in the
			// shaft's mesh, not the shaft getting thinner: the last height is kept for a moment (a real pull-out still
			// closes - the tip comes out past the lips)
			if (through && zTop > zBot && depth > 3.0f && height < 0.5f * r.lastH && g_clock - r.lastThroughAt < 0.35) {
				height = r.lastH;
				width = std::max(width, r.lastW);
			} else if (through && zTop > zBot) {   // 0.4.5
				r.lastH = height;
				r.lastW = width;
				r.lastThroughAt = g_clock;
			}
			const float restWidth = (r.bones[6].valid && r.bones[7].valid) ? len(rest[7] - rest[6]) : 4.0f;
			float want[kMorphs] = {};
			if (through || (inMouth && DeepFit::MegaMouthNow())) {   // 0.3.1: Mega mouth opens fully as soon as it is in his mouth
				// 0.4.4: a knotted schlong all the way in: the knot at his lips opens his mouth further
				const bool knotIn = g_clock - r.shapeAt < 5.0 && r.shape == 4 && depth > cfg.throatFrom + cfg.autoDeep;
				r.knotOpen += ((knotIn ? cfg.shapeKnotOpen : 0.0f) - r.knotOpen) * (1.0f - std::exp(-delta * 4.0f));
				// 0.5.4: "Open the jaw to the shaft's underside" (snouts) - only the lower jaw opens, so it drops as far as the
				// shaft's underside is below the lower lip, not the shaft's whole height: a shaft riding high in a snout (on
				// the upper teeth) left the jaw hanging open far below it
				float body = height;
				if (cfg.jawToUnderside && r.bones[kBottomCenter].valid && zTop > zBot) {
					const float lowRest = dot(rest[kBottomCenter] - C, Z);
					const float under = std::max(zBot, zTop - height);   // (the along-the-shaft cut's height, when it read less)
					body = std::clamp(lowRest - under, 0.0f, height);
				}
				const float open = body + cfg.mouthExtra + r.knotOpen;
				want[0] = DeepFit::MegaMouthNow() ? cfg.morphMax : open / std::max(cfg.jawUnits, 0.1f) * 100.0f;
				want[1] = DeepFit::MegaMouthNow() ? cfg.morphMax * cfg.lipRound : open * 0.5f / std::max(cfg.lipUnits, 0.1f) * 100.0f * cfg.lipRound;
				// 0.3.8: the corners go out to the sides too - to the shaft's width (plus MouthExtra) and a share of how far
				// the jaw is open - so the mouth opens wide round it instead of pushing the lips forward into a pucker
				{
					const float out = std::max(0.0f, 0.5f * (width + cfg.mouthExtra) - 0.5f * restWidth) + cfg.cornerOpen * open;
					want[kMouthWide] = DeepFit::MegaMouthNow() ? 100.0f * cfg.mouthWideMax : 100.0f * std::clamp(out * cfg.mouthWideGain, 0.0f, cfg.mouthWideMax);
					// 0.4.4: the lower lip's sides rounded out as the jaw opens (a U, not a diamond pinched at its sides)
					want[kLowerRound] = 100.0f * cfg.lowerLipRound * std::clamp(want[0] / 100.0f, 0.0f, 1.0f);
					// 0.4.7: a snout's upper jaw comes down onto the shaft as the jaw opens (heads with DF_UpperJawDown)
					want[kUpperJaw] = 100.0f * cfg.upperJawDown * std::clamp(want[0] / 100.0f, 0.0f, 1.0f);
				}
				want[2] = DeepFit::MegaMouthNow() ? cfg.morphMax : std::max(0.0f, (width + cfg.mouthExtra - restWidth) * 0.5f) / std::max(cfg.widenUnits, 0.1f) * 100.0f;
			}
			if (cfg.throatMorphs && (through || inMouth)) {
				// 0.3.4: one smooth ridge instead of three lumps one after another. The three morphs overlap down his
				// throat (from the SMP Head tri): TH1 just under the chin (it pushes down), TH3 the middle, TH2 low by
				// the neck seam. They come out in that order as the tip goes deeper, overlapping, and stay out, each at
				// its own share so they add up to an even ridge - TH2 kept lower so there is no ledge at the neck seam.
				const float u = (depth - cfg.throatFrom) / std::max(cfg.throatFull - cfg.throatFrom, 0.5f);
				auto part = [&](float a, float b, float w) { return std::clamp((u - a) / (b - a), 0.0f, 1.0f) * w * cfg.throatMorphMax; };
				want[3] = part(0.0f, 0.45f, 0.5f);    // TH1
				want[5] = part(0.15f, 0.7f, 1.0f);    // TH3
				want[4] = part(0.35f, 1.0f, 0.7f);    // TH2
			}
			// 0.3.5: the throat follows the schlong's own shape: the shaft is measured every half unit along its length inside
			// his mouth (how thick it is there). Each band of his throat, from ThroatFrom deep on, is filled out to the line
			// once the shaft reaches it, and swells past it by how much thicker than an empty throat (ThroatRest) the part
			// passing it is - a flared head, a medial ring or a knot shows as it goes down
			if (cfg.throatShape && (through || inMouth)) {
				constexpr int kBins = 80;   // half-unit bins, 40 units
				float zLo[kBins], zHi[kBins], xLo[kBins], xHi[kBins];
				for (int b = 0; b < kBins; ++b) { zLo[b] = xLo[b] = 1e9f; zHi[b] = xHi[b] = -1e9f; }
				// 0.4.0: measured along the shaft's own line, not straight back from his lips - a shaft that goes in at an angle
				// or bends read thicker than it is when sliced straight across his mouth. The line: the main direction of
				// the shaft points inside his mouth (or straight back when there are too few of them, or ThroatAxisFit is off)
				NiPoint3 ax(0.0f, -1.0f, 0.0f), org(0.0f, 0.0f, 0.0f);
				if (cfg.throatAxisFit) {
					NiPoint3 m(0, 0, 0); int n = 0;
					for (auto& p : hs)
						if (p.y < 0.5f && p.y > -40.0f && std::fabs(p.x) < 8.0f && std::fabs(p.z) < 8.0f) { m = m + p; ++n; }
					if (n >= 24) {
						m = m * (1.0f / n);
						float cxx = 0, cxy = 0, cxz = 0, cyy = 0, cyz = 0, czz = 0;
						for (auto& p : hs)
							if (p.y < 0.5f && p.y > -40.0f && std::fabs(p.x) < 8.0f && std::fabs(p.z) < 8.0f) {
								const NiPoint3 d = p - m;
								cxx += d.x * d.x; cxy += d.x * d.y; cxz += d.x * d.z; cyy += d.y * d.y; cyz += d.y * d.z; czz += d.z * d.z;
							}
						NiPoint3 v(0.0f, -1.0f, 0.0f);
						for (int it = 0; it < 12; ++it) {   // the main direction (power iteration)
							NiPoint3 w(cxx * v.x + cxy * v.y + cxz * v.z, cxy * v.x + cyy * v.y + cyz * v.z, cxz * v.x + cyz * v.y + czz * v.z);
							const float lw = len(w);
							if (lw < 1e-6f) break;
							v = w * (1.0f / lw);
						}
						if (v.y > 0.0f) v = v * -1.0f;
						if (v.y < -0.5f) {   // within ~60 degrees of straight back: use it
							ax = v;
							org = m - ax * (m.y / ax.y);   // where that line crosses his lips
						}
					}
				}
				// across the line: sideways (his right) and up, square to it
				NiPoint3 ux = NiPoint3(1.0f, 0.0f, 0.0f) - ax * ax.x;
				ux = len(ux) > 1e-4f ? norm(ux) : NiPoint3(1.0f, 0.0f, 0.0f);
				const NiPoint3 uz = norm(NiPoint3(ax.y * ux.z - ax.z * ux.y, ax.z * ux.x - ax.x * ux.z, ax.x * ux.y - ax.y * ux.x)) * -1.0f;
				for (auto& p : hs) {
					const NiPoint3 q = p - org;
					const float sAlong = dot(q, ax);
					const NiPoint3 d = q - ax * sAlong;
					const float px = dot(d, ux), pz = dot(d, uz);
					if (px * px + pz * pz > 64.0f)
						continue;   // not on the shaft's line (the balls, his partner's belly)
					const int b = static_cast<int>(std::floor(sAlong * 2.0f));
					if (b < 0 || b >= kBins)
						continue;
					zLo[b] = std::min(zLo[b], pz); zHi[b] = std::max(zHi[b], pz);
					xLo[b] = std::min(xLo[b], px); xHi[b] = std::max(xHi[b], px);
				}
				// the shaft's half height (towards the front of his throat) and half width (across it) s units past the lips
				auto halfH = [&](float s) {
					const int b = static_cast<int>(std::floor(s * 2.0f));
					return (b < 0 || b >= kBins || zHi[b] < zLo[b]) ? 0.0f : 0.5f * (zHi[b] - zLo[b]);
				};
				auto halfW = [&](float s) {
					const int b = static_cast<int>(std::floor(s * 2.0f));
					return (b < 0 || b >= kBins || xHi[b] < xLo[b]) ? 0.0f : 0.5f * (xHi[b] - xLo[b]);
				};
				float tip = -1.0f;   // how far in the tip is (the deepest bin with shaft in it)
				for (int b = kBins - 1; b >= 0; --b)
					if (zHi[b] >= zLo[b]) { tip = (b + 1) * 0.5f; break; }
				// 0.4.4: an extra push - the whole shaft read that much deeper, so the bulge goes further down his throat
				const float ex = tip > 0.0f ? r.extraCur : 0.0f;
				if (ex > 0.0f)
					tip += ex;
				float raw[kBands], reachK[kBands];
				for (int k = 0; k < kBands; ++k) {
					const float s = cfg.throatFrom + (k + 0.5f) * cfg.throatBand;
					const float lo = s - 0.25f * cfg.throatBand, hi = s + 0.25f * cfg.throatBand;
					const float lo2 = std::max(lo - ex, 0.25f), hi2 = std::max(hi - ex, 0.25f);
					const float rz = std::max(halfH(lo2), halfH(hi2)), rx = std::max(halfW(lo2), halfW(hi2));
					// eased in over the last three bands before the tip: a rounded end, not a cut-off (0.3.8: the swelling
					// tapers there too - it used to stay full right up to the tip, a spike in the middle of his throat)
					float reach = std::clamp((tip - (s - 0.5f * cfg.throatBand)) / (3.0f * cfg.throatBand), 0.0f, 1.0f);
					reach = reach * reach * (3.0f - 2.0f * reach);
					const bool here = rz > 0.0f && reach > 0.0f;
					want[kFill0 + k] = here ? 100.0f * cfg.throatFill * reach : 0.0f;
					raw[k] = here ? std::clamp((rz - cfg.throatRest) * cfg.throatShapeGain, 0.0f, cfg.throatShapeMax) : 0.0f;
					reachK[k] = here ? reach : 0.0f;
					// wider than it is tall: the bulge spreads sideways (not further out)
					want[kWide0 + k] = here ? 100.0f * reach * std::clamp(cfg.throatWideExtra + (rx - std::max(rz, cfg.throatRest)) / std::max(cfg.throatWideUnits, 0.1f), 0.0f, 1.0f) : 0.0f;
				}
				// 0.3.5d: the shape stands out - each band's difference from the shaft's usual thickness (the middle value
				// of the bands it fills) is made ThroatDetail times bigger, so a flare, ring or knot reads clearly
				{
					float v[kBands]; int n = 0;
					for (int k = 0; k < kBands; ++k) if (raw[k] > 0.0f) v[n++] = raw[k];
					if (n > 0 && cfg.throatDetail != 1.0f) {
						std::sort(v, v + n);
						const float mid = v[n / 2];
						for (int k = 0; k < kBands; ++k)
							if (raw[k] > 0.0f) raw[k] = std::clamp(mid + (raw[k] - mid) * cfg.throatDetail, 0.0f, cfg.throatShapeMax);
					}
				}
				// 0.4.4: a horse's flared head shows down his throat even when his mesh does not read as flared (a shaft
				// built round his bones, a plain mesh): it swells just behind the tip
				if (g_clock - r.shapeAt < 5.0 && r.shape == 3 && cfg.shapeFlare > 0.0f && tip > 0.0f)
					for (int k = 0; k < kBands; ++k) {
						const float s = cfg.throatFrom + (k + 0.5f) * cfg.throatBand;
						const float d = tip - s;   // how far behind the tip
						if (reachK[k] > 0.0f)
							raw[k] = std::min(raw[k] + cfg.shapeFlare * std::exp(-std::pow((d - 0.9f) / 0.8f, 2.0f)), cfg.throatShapeMax + cfg.shapeFlare);
					}
				// 0.4.5: an extra push also swells the throat for a moment, so it shows even when the shaft already fills it
				if (r.extraCur > 0.0f && cfg.extraDepth > 0.0f)
					for (int k = 0; k < kBands; ++k)
						if (reachK[k] > 0.0f)
							raw[k] = std::min(raw[k] + cfg.extraSwell * (r.extraCur / cfg.extraDepth), cfg.throatShapeMax + cfg.extraSwell);
				for (int k = 0; k < kBands; ++k)   // 0.3.8: tapered towards the tip (after the shape is brought out)
					raw[k] *= reachK[k];
				// 0.4.0: what measuring cannot see - a ridged schlong's ridge, a segmented one's knobs (Anatomica says which,
				// by mod event; the test page sets it too)
				const bool shapeFresh = g_clock - r.shapeAt < 5.0;
				for (int k = 0; k < kBands; ++k) {
					const float f = want[kFill0 + k] / 100.0f;
					want[kRidge0 + k] = (shapeFresh && r.shape == 6) ? 100.0f * cfg.shapeRidge * f : 0.0f;
					want[kKnob0 + k] = (shapeFresh && r.shape == 7 && (k % 2) == 0) ? 100.0f * cfg.shapeKnob * f : 0.0f;
				}
				for (int k = 0; k < kBands; ++k)   // a touch of blur along it, so a sudden step in the shaft is not a shelf
					want[kBand0 + k] = 100.0f * (0.1f * raw[std::max(k - 1, 0)] + 0.8f * raw[k] + 0.1f * raw[std::min(k + 1, kBands - 1)]);
				// 0.3.6: it keeps moving with every stroke, even when the throat is as full as it gets: a soft ripple tied to the
				// tip (it slides along his throat as the shaft goes in and out), on top of the shape and past its cap
				if (cfg.throatMotion > 0.0f && tip > 0.0f) {
					const float lam = std::max(cfg.throatMotionLen, 0.5f);
					for (int k = 0; k < kBands; ++k) {
						if (want[kFill0 + k] <= 0.0f)
							continue;
						const float s = cfg.throatFrom + (k + 0.5f) * cfg.throatBand;
						want[kBand0 + k] += 100.0f * cfg.throatMotion * (want[kFill0 + k] / 100.0f) * (0.5f + 0.5f * std::cos(6.2831853f * (s - tip) / lam));
					}
				}
				// 0.3.6: no band more than 0.45 units (0.3.8; was 0.7) above its neighbour - a big flare or knot rises over a couple of bands
				// instead of a spike (the head's mesh is too coarse there for a sudden step)
				for (int pass = 0; pass < 2; ++pass) {
					for (int k = 1; k < kBands; ++k) want[kBand0 + k] = std::min(want[kBand0 + k], want[kBand0 + k - 1] + 45.0f);
					for (int k = kBands - 2; k >= 0; --k) want[kBand0 + k] = std::min(want[kBand0 + k], want[kBand0 + k + 1] + 45.0f);
				}
				for (int k = 0; k < kBands; ++k)   // 0.3.6: the swelling spread across the wider tube as well (its width x its height)
					want[kWideT0 + k] = want[kWide0 + k] / 100.0f * want[kBand0 + k];
			}
			// 0.4.0: the partner's race strength (the throat's fill to the line is kept as it is)
			if (r.raceK != 1.0f)
				for (int m = 0; m < kMorphs; ++m)
					if (m < kFill0 || m >= kFill0 + kBands)
						want[m] *= r.raceK;
			// 0.4.0: how the shaft moves in his mouth: in (+) or out (-), units a second
			r.full = through || inMouth;
			if (r.full) {
				if (r.depthPrev >= 0.0f && delta > 0.0f)
					r.vel += ((depth - r.depthPrev) / delta - r.vel) * (1.0f - std::exp(-delta * 12.0f));
				r.depthPrev = depth;
				r.wasIn = true;
				r.leftJaw = r.morph[0];
				const float dk = std::clamp((depth - 2.0f) / std::max(cfg.throatFrom + 4.0f, 1.0f), 0.0f, 1.0f);
				r.depthK += (dk - r.depthK) * (1.0f - std::exp(-delta * 6.0f));
				// 0.4.4: an extra push - at the deepest point of a stroke (pushing in, then stopping), now and then (a chance
				// each stroke) it goes ExtraDepth further for a moment
				if (cfg.extraPush && r.prevVel > 3.0f && r.vel <= 0.5f && depth > cfg.throatFrom &&
					std::uniform_real_distribution<float>(0.0f, 1.0f)(g_rng) < cfg.extraChance)
					r.extraUntil = g_clock + cfg.extraHold;
				r.prevVel = r.vel;
				// a gag or a swallow by itself (Anatomica's own, when it sends them, come first)
				if (cfg.autoGagGulp && g_clock - r.extAt > 10.0) {
					const bool deep = depth > cfg.throatFrom + cfg.autoDeep;
					if (deep && r.vel > cfg.autoGagSpeed && g_clock - r.gagAt > cfg.autoGagCooldown) {
						r.gagAt = g_clock;
						r.gagK = std::clamp(r.vel / std::max(cfg.autoGagSpeed, 1.0f) * 0.7f, 0.5f, 1.5f);
					}
					if (deep && std::fabs(r.vel) < 4.0f) {
						r.holdT += delta;
						if (r.holdT > cfg.autoGulpHold && g_clock - r.gulpAt > cfg.autoGulpHold + 1.0) {
							r.gulpAt = g_clock;
							r.holdT = 0.0f;
						}
					} else
						r.holdT = 0.0f;
				}
			} else {
				r.depthPrev = -1.0f;
				r.vel *= std::exp(-delta * 6.0f);
				r.depthK *= std::exp(-delta * 4.0f);
			}
			// 0.4.0: after he pulls out (still close by) the mouth stays a little open for a moment, then he swallows
			if (!r.full) {
				if (r.wasIn) {
					r.wasIn = false;
					r.leftAt = g_clock;
					r.recoverGulped = false;
				}
				const double tl = g_clock - r.leftAt;
				if (cfg.recoverHold > 0.0f && tl < cfg.recoverHold)
					want[0] = std::max(want[0], r.leftJaw * cfg.recoverOpen);
				if (!r.recoverGulped && cfg.recoverGulp && tl > cfg.recoverHold * 0.6 && tl < 5.0) {
					r.recoverGulped = true;
					if (g_clock - r.gulpAt > 1.0)
						r.gulpAt = g_clock;
				}
			}
			{   // 0.4.4: the extra push eases in quickly and back out slowly
				const float to = (cfg.extraPush && r.full && g_clock < r.extraUntil) ? cfg.extraDepth : 0.0f;
				r.extraCur += (to - r.extraCur) * (1.0f - std::exp(-delta * (to > r.extraCur ? 10.0f : 3.0f)));
				if (r.extraCur < 0.01f)
					r.extraCur = 0.0f;
			}
			// the lips drag with it: rolled out as he pulls back, tucked in as he pushes in
			r.dragWant = r.full ? std::clamp(-r.vel / std::max(cfg.lipDragSpeed, 1.0f), -1.0f, 1.0f) * cfg.lipDrag : 0.0f;
			const float km = 1.0f - std::exp(-delta * 10.0f);
			for (int m = 0; m < kMorphs; ++m) {
				want[m] = std::clamp(want[m], 0.0f, cfg.morphMax * (throatScaled(m) ? kThroatScale : 1.0f));
				// 0.5.2: "Pull-out hold" - the jaw and lip morphs close at most this fast (all the way in that many seconds):
				// as he pulls back, the thin tip at the lips no longer snaps the mouth shut and open again
				if (cfg.pullOutHold > 0.0f && (m == 0 || m == 1 || m == 2 || m == kMouthWide) && want[m] < r.morph[m])
					want[m] = std::max(want[m], r.morph[m] - 100.0f * delta / cfg.pullOutHold);
				// 0.5.4: "Opening ease" - the other way: as the tip goes in, the jaw opens no faster than this (all the way
				// in that many seconds), so it does not jump open in one frame
				if (cfg.openEase > 0.0f && (m == 0 || m == 1 || m == 2 || m == kMouthWide) && want[m] > r.morph[m])
					// 0.5.7: faster the further behind it is (a thick part arriving pushed through the lips while it eased)
					want[m] = std::min(want[m], r.morph[m] + 100.0f * delta / cfg.openEase * (1.0f + 3.0f * std::clamp((want[m] - r.morph[m]) / 100.0f, 0.0f, 1.0f)));
				r.morph[m] += (want[m] - r.morph[m]) * km;
			}
			r.teeth = cfg.teethOpen && (through || inMouth);
			// what the morphs did to each lip (their size at 100, from the SMP Head tri): the bones only make up the rest
			// 0.4.8: the lower lip and the corners by how far the head's morphs really move them (measured on the head), not by
			// JawUnits / WidenUnits - those set how wide the mouth opens (people tune them low for a wider mouth), and read as
			// sizes they made DeepFit think the lower lip had dropped less than it had (it stayed down, short of the shaft) and
			// the corners were further out than they were (pulled in ~2 units off the shaft's sides). The upper lip as before.
			const float round = r.morph[1] / 100.0f * cfg.lipUnits;
			const float j01 = std::clamp(r.morph[0] / 100.0f, 0.0f, 1.0f), r01 = std::clamp(r.morph[1] / 100.0f, 0.0f, 1.0f);
			const float w01 = std::clamp(r.morph[2] / 100.0f, 0.0f, 1.0f), mw01 = std::clamp(r.morph[kMouthWide] / 100.0f, 0.0f, 3.0f);
			const float drop = headJawDrop(r);   // the lower lip's drop with the jaw fully open (4.1 on the human head)
			for (int i = 0; i < r.nspecs(); ++i) {
				if (r.specs[i].kind == Kind::Lip) {
					const NiPoint3 d = norm(r.dir(i));
					comp[i] = d.z > 0 ? round * 0.5f : j01 * drop * (i == kBottomCenter ? 1.0f : 0.87f) * std::fabs(d.z);
				} else if (r.specs[i].kind == Kind::Corner)   // PPA_LipWiden 1.06 out, DF_MouthWide 1.0 out, the lip-round 0.87 in, the jaw 0.31 in
					comp[i] = std::max(0.0f, 1.06f * w01 + mw01 - 0.87f * r01 - 0.31f * j01);
			}
			if (cfg.debugLog && g_logTimer > 1.0f) {   // 0.2.2: where the shaft is, from his mouth (right / forward / up)
				const NiPoint3* nearest = nullptr;
				float           best = 1e9f;
				for (auto& p : hs) {
					const float d = p.x * p.x + p.y * p.y + p.z * p.z;
					if (d < best) {
						best = d;
						nearest = &p;
					}
				}
				if (nearest)
					logs::info("DeepFit: nearest shaft point {:.1f} from the mouth centre (right {:.1f}, forward {:.1f}, up {:.1f}); lips {:.1f} apart",
						std::sqrt(best), nearest->x, nearest->y, nearest->z,
						(r.bones[kUpperCenter].valid && r.bones[kBottomCenter].valid) ? len(rest[kUpperCenter] - rest[kBottomCenter]) : -1.0f);
			}
			if (cfg.debugLog && g_logTimer > 1.0f)
				logs::info("DeepFit: mouth - shaft {:.2f} tall, {:.2f} wide at the lips, tip {:.1f} deep | jaw {:.0f} round {:.0f} widen {:.0f} throat {:.0f}/{:.0f}/{:.0f} bands {:.0f}/{:.0f}/{:.0f}/{:.0f}/{:.0f}/{:.0f}/{:.0f}/{:.0f}/{:.0f}/{:.0f}{}",
					height, width, depth, r.morph[0], r.morph[1], r.morph[2], r.morph[3], r.morph[4], r.morph[5],
					r.morph[kBand0], r.morph[kBand0 + 1], r.morph[kBand0 + 2], r.morph[kBand0 + 3], r.morph[kBand0 + 4],
					r.morph[kBand0 + 5], r.morph[kBand0 + 6], r.morph[kBand0 + 7], r.morph[kBand0 + 8], r.morph[kBand0 + 9],
					DeepFit::MegaMouthNow() ? " (mega mouth)" : "");
		}
		for (int i = 0; i < r.nspecs(); ++i) {
			auto& b = r.bones[i];
			if (!b.valid)
				continue;
			const auto& spec = r.specs[i];
			NiPoint3    rel = rest[i] - C;
			NiPoint3    rh(dot(rel, X), dot(rel, Y), dot(rel, Z));

			if (spec.kind == Kind::Lip || spec.kind == Kind::Corner) {
				NiPoint3 d = norm(r.dir(i));
				NiPoint3 e(d.z, 0.0f, -d.x);  // across the lip, in the mouth plane
				float    sRest = rh.x * d.x + rh.z * d.z;
				float    sMax = -1e9f;
				const float mxLip = spec.kind == Kind::Corner ? cfg.cornerMax : (r.dir(i).z > 0 ? cfg.upperLipMax : cfg.lowerLipMax);
				// 0.4.5: a low-poly shaft (a werewolf's, ~200 vertices) has rings further apart than the slab: the lip's
				// highest point then jumped from ring to ring as it slid and the upper lip jittered. Few points found = look
				// wider (and the lips ease more slowly below)
				int hits = 0;
				auto scan = [&](float slab, float lane) {
					for (auto& p : hs) {
						// 0.5.1: the lower lip also feels the shaft a little further into the mouth ("Lower lip follows the shaft
						// in"): a shaft sloping down into the mouth then draws the front of the lower lip (and the jaw) down to it
						const float in = (d.z < 0.0f && spec.kind == Kind::Lip) ? cfg.lowerLipInward : 0.0f;
						if (p.y > rh.y + slab || p.y < rh.y - slab - in)
							continue;
						if (std::fabs(p.x * e.x + p.z * e.z) > lane)
							continue;
						float s = p.x * d.x + p.z * d.z;
						if (s - sRest > mxLip * 2.0f + 1.0f)
							continue;   // 0.1.4: far past what this lip can reach (the shaft's base, the balls) - not round the lip
						++hits;
						if (s > sMax)
							sMax = s;
					}
				};
				scan(cfg.lipDepthSlab, cfg.lipLaneWidth);
				if (hits < 3) {
					hits = 0;
					sMax = -1e9f;
					scan(cfg.lipDepthSlab * 2.5f, cfg.lipLaneWidth * 2.0f);
				}
				float mx = mxLip;
				if (DeepFit::MouthNow()) {   // 0.2.0: the morphs opened the mouth; the bones fit the lips to the shaft (out, or back in round a slim one)
					// (0.4.2: closing in round a slim shaft only when it is in his mouth - a shaft just under his chin pulled
					// his upper lip 5 units in)
					target[i] = sMax > -1e8f ? std::clamp(sMax + cfg.lipMargin + cfg.stretch - sRest - comp[i], (DeepFit::MegaMouthNow() || !inMouth) ? 0.0f : -(spec.kind == Kind::Corner && cfg.cornerCloseMax >= 0.0f ? cfg.cornerCloseMax : cfg.lipCloseMax), mx) : 0.0f;
					// 0.3.3: the lip-round morph peaks the middle of the upper lip up towards the nose - pull it back down
					// 0.4.5: the upper lip down onto a shaft that sits lower than it (the thin front of the shaft rides low in his
					// mouth on a shallow stroke) - harder, as its bones are only half weighted
					if (spec.kind == Kind::Lip && r.dir(i).z > 0.0f && inMouth && target[i] < 0.0f && cfg.upperLipClose != 1.0f)
						target[i] *= cfg.upperLipClose;
					// 0.4.9: and the lower lip up onto it (the jaw opened wider than the shaft - a gap stayed under a canine's)
					if (spec.kind == Kind::Lip && r.dir(i).z < 0.0f && inMouth && target[i] < 0.0f && cfg.lowerLipClose != 1.0f)
						target[i] *= cfg.lowerLipClose;
					// 0.5.2: and the corners in onto its sides (a cat's corner bones carry little of the mesh - the sides stayed open)
					if (spec.kind == Kind::Corner && inMouth && target[i] < 0.0f && cfg.cornerClose != 1.0f)
						target[i] *= cfg.cornerClose;
					// (0.4.7: below 0 lifts it instead - a cat's muzzle rises off the shaft as the jaw opens wide)
					// (0.4.9: a lift only while the shaft is under that lip, and as far as the jaw is open - pulling out, the lip stayed
					// lifted with nothing under it and folded up over an Argonian's snout)
					if (spec.kind == Kind::Lip && r.dir(i).z > 0.0f && inMouth && cfg.upperLipDown < 0.0f && sMax > -1e8f)
						target[i] = std::min(target[i] - cfg.upperLipDown * (i == kUpperCenter ? 1.0f : 0.6f) * std::clamp(r.morph[0] / 100.0f, 0.0f, 1.0f), mx - cfg.upperLipDown);
					else if (spec.kind == Kind::Lip && r.dir(i).z > 0.0f && inMouth && cfg.upperLipDown > 0.0f)
						target[i] = std::min(target[i] - cfg.upperLipDown * (i == kUpperCenter ? 1.0f : 0.6f), mx);
					// 0.4.1: and the lower lip up (the jaw morph drops it further than the shaft needs)
					if (spec.kind == Kind::Lip && r.dir(i).z < 0.0f && inMouth && cfg.lowerLipUp > 0.0f)
						target[i] -= cfg.lowerLipUp * (i == kBottomCenter ? 1.0f : 0.6f);
				} else {
					target[i] = sMax > -1e8f ? std::clamp(sMax + cfg.lipMargin + cfg.stretch - sRest, 0.0f, mx) : 0.0f;
					if (inMouth)
						target[i] = std::min(std::max(target[i], spec.kind == Kind::Corner ? cfg.minCorner : cfg.minOpen), mx);
				}
			} else if (spec.kind == Kind::Cheek) {
				// 0.4.0: a shaft pushed to his side of the mouth (at the cheek's depth) bulges that cheek out
				if (cfg.cheekGain <= 0.0f)
					continue;
				const float sgn = r.dir(i).x < 0.0f ? -1.0f : 1.0f;
				float sMax = 0.0f;
				for (auto& p : hs) {
					if (p.y > -0.5f || std::fabs(p.y - rh.y) > 2.5f || std::fabs(p.z - rh.z) > 2.5f)
						continue;   // inside his mouth, about as deep as the cheek
					sMax = std::max(sMax, p.x * sgn - cfg.cheekInner);
				}
				target[i] = std::clamp(sMax * cfg.cheekGain, 0.0f, cfg.cheekMax);
			} else if (spec.kind == Kind::Throat) {
				if (!cfg.throatEnabled)
					continue;
				float sMax = -1e9f;
				for (auto& p : hs) {
					if (std::fabs(p.z - rh.z) > cfg.throatSlabZ || std::fabs(p.x - rh.x) > cfg.throatSlabX)
						continue;
					float s = p.y - rh.y;
					if (s > sMax)
						sMax = s;
				}
				float raw = sMax > -1e8f ? (sMax - cfg.throatSkinOffset + cfg.throatMargin) : 0.0f;
				target[i] = std::clamp(raw * cfg.throatGain, 0.0f, cfg.throatMax);
			}
		}
		// 0.4.9: each lip's bones smoothed against their neighbours along it - a snout has more lip bones than a man's mouth,
		// and one pushed further than the next made a spike at the side of the mouth (Argonian, Khajiit)
		if (cfg.lipSmooth > 0.0f) {
			for (int upper = 0; upper < 2; ++upper) {
				int   idx[kMaxSpecs];
				float xs[kMaxSpecs];
				int   n = 0;
				for (int i = 0; i < r.nspecs(); ++i) {
					if (!r.bones[i].valid || r.specs[i].kind != Kind::Lip || (r.dir(i).z > 0.0f) != (upper == 1))
						continue;
					idx[n] = i;
					xs[n] = dot(rest[i] - C, X);
					++n;
				}
				for (int a = 1; a < n; ++a)   // along the lip, left to right
					for (int b = a; b > 0 && xs[b] < xs[b - 1]; --b) {
						std::swap(xs[b], xs[b - 1]);
						std::swap(idx[b], idx[b - 1]);
					}
				if (n < 4)   // (people's lips - three bones each - left as they were)
					continue;
				float sm[kMaxSpecs];
				for (int pass = 0; pass < 2; ++pass) {
					for (int k = 0; k < n; ++k) {
						const float t = target[idx[k]];
						const float l = target[idx[k > 0 ? k - 1 : k]], rr = target[idx[k < n - 1 ? k + 1 : k]];
						sm[k] = t + ((l + rr) * 0.5f - t) * 0.5f * cfg.lipSmooth;
					}
					for (int k = 0; k < n; ++k)
						target[idx[k]] = sm[k];
				}
			}
		}
		// 0.5.1: the whole lower jaw drops as far as the front of the lower lip has to go down (never up) - the jaw unhinges
		// a little for a shaft that slopes down into the mouth, instead of the lip alone stretching
		// (three bones along the jaw - front, middle, back - each as far as the length of its direction in the head profile
		// says: 1 at the front, less further back, so the jaw bends a little instead of dropping as one piece)
		for (int i = 0; i < r.nspecs(); ++i)
			if (r.specs[i].kind == Kind::Jaw && r.bones[i].valid) {
				const float share = std::clamp(len(r.specs[i].dir), 0.0f, 1.0f);
				// (0.5.2: up as well as down - the front of the jaw folds up onto the shaft's underside when the jaw opened
				// further than the shaft needs, instead of hanging below it)
				target[i] = (DeepFit::MouthNow() && r.bones[kBottomCenter].valid) ? std::clamp(target[kBottomCenter] * cfg.jawFollow, -cfg.jawFollowMax, cfg.jawFollowMax) * share : 0.0f;
				// 0.5.3: "Lower jaw fits the shaft's slope" - each jaw bone also finds the shaft's underside right above it
				// (at its own depth in the mouth) and goes to it: the front of the jaw bends up onto a shaft that slopes up
				// out of the mouth, the back stays down where the shaft is lower, instead of all three following the lip
				if (cfg.jawFitShaft > 0.0f && DeepFit::MouthNow() && inMouth) {
					const NiPoint3 rel = rest[i] - C;
					const NiPoint3 rh(dot(rel, X), dot(rel, Y), dot(rel, Z));
					float          zMin = 1e9f;
					int            hits = 0;
					for (auto& p : hs) {
						if (std::fabs(p.y - rh.y) > cfg.lipDepthSlab * 2.5f || std::fabs(p.x - rh.x) > cfg.lipLaneWidth * 2.0f)   // (0.5.5: wider - a ring of the shaft's mesh coming and going made the jaw flicker)
							continue;
						if (p.z < rh.z - cfg.jawFollowMax - 6.0f)
							continue;   // far below anything the jaw could reach
						zMin = std::min(zMin, p.z);
						++hits;
					}
					if (hits >= 2) {
						const NiPoint3 dn = norm(r.specs[i].dir);
						// where the jaw morph left the floor of the mouth here, against the shaft's underside
						const float floorZ = rh.z - std::clamp(r.morph[0] / 100.0f, 0.0f, 1.0f) * headJawDrop(r) * share;
						const float down = floorZ - (zMin - cfg.lipMargin);   // > 0: the shaft is lower, the jaw goes down to it
						const float fit = std::clamp(down / std::max(std::fabs(dn.z), 0.3f), -cfg.jawFollowMax, cfg.jawFollowMax) * std::max(share, 0.5f);
						target[i] += (fit - target[i]) * std::clamp(cfg.jawFitShaft, 0.0f, 1.0f);
					}
				}
			}
		// 0.5.4: the tongue (DeepFit's mouths): pushed down as far as the shaft's underside is below its top, between the
		// middle and the front of the lower jaw
		r.tongueWant = 0.0f;
		r.tongueLicks = false;
		if (cfg.tonguePush > 0.0f && DeepFit::MouthNow() && inMouth) {
			int jf = -1, jm = -1;
			for (int i = 0; i < r.nspecs(); ++i)
				if (r.specs[i].kind == Kind::Jaw && r.bones[i].valid) {
					if (r.specs[i].name == "Lower Jaw Front") jf = i;
					else if (r.specs[i].name == "Lower Jaw Mid") jm = i;
				}
			if (jf >= 0 && jm >= 0) {
				const NiPoint3 at = (rest[jf] + rest[jm]) * 0.5f - C;
				const NiPoint3 rh(dot(at, X), dot(at, Y), dot(at, Z));
				float          zMin = 1e9f;
				int            hits = 0;
				for (auto& p : hs)
					if (std::fabs(p.y - rh.y) < 2.0f && std::fabs(p.x - rh.x) < 1.5f && p.z > rh.z - 8.0f) {
						zMin = std::min(zMin, p.z);
						++hits;
					}
				if (hits >= 2) {
					// the floor here after the jaw morph and the jaw bones, and the tongue lying on it
					const float drop = std::clamp(r.morph[0] / 100.0f, 0.0f, 1.0f) * headJawDrop(r) * 0.75f + 0.5f * (target[jf] + target[jm]) * 0.9f;
					const float top = rh.z - drop + cfg.tongueHeight;
					// (0.5.5: and up to it, when the shaft is above it - the tongue stays on the shaft's underside)
					r.tongueWant = std::clamp((top - (zMin - cfg.lipMargin)) * cfg.tonguePush, -cfg.tongueUp, cfg.tongueMax);
					r.tongueLicks = true;
				}
			}
		}
		// nose base follows the upper lip centre
		if (r.bones[8].valid)
			target[8] = std::clamp(target[kUpperCenter] * cfg.noseFollow, 0.0f, cfg.noseMax);

		// 0.5.2: "Pull-out hold" - the lips, corners and jaw bones close in at most this fast (4 units in that many seconds);
		// they still open at once round a thicker part
		if (cfg.pullOutHold > 0.0f)
			for (int i = 0; i < r.nspecs(); ++i) {
				const auto kd = r.specs[i].kind;
				if (r.bones[i].valid && (kd == Kind::Lip || kd == Kind::Corner || kd == Kind::Jaw) && target[i] < r.bones[i].cur)
					target[i] = std::max(target[i], r.bones[i].cur - 4.0f * delta / cfg.pullOutHold);
			}
		// 0.5.4: and the jaw bones open no faster than "Opening ease"; 0.5.5: the lips and corners too - going back in, the
		// tip reaching his lips made every lip jump open to "Lips open at least" in one frame (the jerk at the start of each stroke)
		if (cfg.openEase > 0.0f)
			for (int i = 0; i < r.nspecs(); ++i) {
				const auto kd = r.specs[i].kind;
				if (r.bones[i].valid && (kd == Kind::Jaw || kd == Kind::Lip || kd == Kind::Corner) && target[i] > r.bones[i].cur)
					target[i] = std::min(target[i], r.bones[i].cur + 4.0f * delta / cfg.openEase * (1.0f + 1.5f * std::max(target[i] - r.bones[i].cur, 0.0f)));   // 0.5.7: catches up when far behind
			}
		// smooth + apply
		const float k = 1.0f - std::exp(-delta * cfg.smoothSpeed);
		// 0.4.5: a low-poly shaft: the lips ease more slowly (its few vertices step from frame to frame)
		const float kLip = hs.size() < 600 ? 1.0f - std::exp(-delta * cfg.smoothSpeed * 0.4f) : k;
		r.drag += (r.dragWant - r.drag) * (1.0f - std::exp(-delta * 8.0f));
		for (int i = 0; i < r.nspecs(); ++i) {
			auto& b = r.bones[i];
			if (!b.valid)
				continue;
			b.cur += (target[i] - b.cur) * ((r.specs[i].kind == Kind::Lip || r.specs[i].kind == Kind::Corner) ? kLip : k);
			if (std::fabs(b.cur) < 1e-4f)
				b.cur = 0.0f;
			NiPoint3 d = norm(r.dir(i));
			NiPoint3 offWorld = (X * d.x + Y * d.y + Z * d.z) * b.cur;
			if (r.specs[i].kind == Kind::Lip || r.specs[i].kind == Kind::Corner)   // 0.4.0: the lips drag with the stroke
				offWorld = offWorld + Y * (r.drag * (r.specs[i].kind == Kind::Corner ? 0.4f : 1.0f));
			const NiTransform& P = b.parent->world;
			NiPoint3 offLocal = P.rotate.Transpose() * offWorld;
			if (P.scale > 1e-6f)
				offLocal = offLocal * (1.0f / P.scale);
			b.node->local.translate = b.restLocal + offLocal;
			b.node->world = P * b.node->local;
		}

		if (cfg.debugLog && g_logTimer > 1.0f) {
			logs::info("DeepFit: shaft verts {} | upper {:.2f} lower {:.2f} cornerL {:.2f} nose {:.2f} throat {:.2f}/{:.2f}/{:.2f}",
				shaft.size(), r.bones[0].cur, r.bones[3].cur, r.bones[6].cur, r.bones[8].cur, r.bones[9].cur, r.bones[10].cur, r.bones[11].cur);
		}
	}

	void relax(Receiver& r, float delta)
	{
		auto& cfg = cfgOf(r);
		r.full = false;
		r.shaftWorld.clear();   // 0.5.9
		r.depthPrev = -1.0f;
		r.vel *= std::exp(-delta * 6.0f);
		r.depthK *= std::exp(-delta * 4.0f);
		r.dragWant = 0.0f;
		r.drag -= r.drag * (1.0f - std::exp(-delta * 8.0f));
		// 0.4.0: after he pulls out the mouth stays a little open for a moment, then he swallows
		if (r.wasIn) {
			r.wasIn = false;
			r.leftAt = g_clock;
			r.recoverGulped = false;
		}
		const double tl = g_clock - r.leftAt;
		const bool   holding = cfg.recoverHold > 0.0f && tl < cfg.recoverHold;
		if (!r.recoverGulped && cfg.recoverGulp && tl > cfg.recoverHold * 0.6 && tl < 5.0) {
			r.recoverGulped = true;
			if (g_clock - r.gulpAt > 1.0)
				r.gulpAt = g_clock;
		}
		{   // 0.2.0: no shaft: the mouth morphs ease back to nothing
			const float km = 1.0f - std::exp(-delta * 6.0f);
			for (int m = 0; m < kMorphs; ++m) {
				const float to = (m == 0 && holding) ? r.leftJaw * cfg.recoverOpen : 0.0f;
				r.morph[m] += (to - r.morph[m]) * (m == 0 && holding ? 1.0f - std::exp(-delta * 3.0f) : km);
			}
			r.teeth = false;
		}
		// no shaft nearby: ease every bone back to rest
		const float k = 1.0f - std::exp(-delta * Config::get().smoothSpeed);
		for (int i = 0; i < r.nspecs(); ++i) {
			auto& b = r.bones[i];
			if (!b.valid || (b.cur == 0.0f && r.drag == 0.0f))
				continue;
			b.cur *= (1.0f - k);
			if (std::fabs(b.cur) < 1e-4f)
				b.cur = 0.0f;
			if (std::fabs(r.drag) < 1e-4f)
				r.drag = 0.0f;
			NiPoint3 d = norm(r.dir(i));
			NiPoint3 offWorld = (r.fx * d.x + r.fy * d.y + r.fz * d.z) * b.cur;   // 0.1.3: the face's own axes
			if (r.specs[i].kind == Kind::Lip || r.specs[i].kind == Kind::Corner)
				offWorld = offWorld + r.fy * (r.drag * (r.specs[i].kind == Kind::Corner ? 0.4f : 1.0f));
			const NiTransform& P = b.parent->world;
			NiPoint3 offLocal = P.rotate.Transpose() * offWorld;
			if (P.scale > 1e-6f)
				offLocal = offLocal * (1.0f / P.scale);
			b.node->local.translate = b.restLocal + offLocal;
			b.node->world = P * b.node->local;
		}
	}

	// ------------------------------------------------------------------ hook

	// 0.3.0: switched from outside (the key, other mods' mod events): -1 = as the setting says
	std::atomic<int> g_megaOverride{ -1 };
	std::atomic<int> g_mouthOverride{ -1 };

	class ModEvents : public BSTEventSink<SKSE::ModCallbackEvent>
	{
	public:
		static ModEvents* Get()
		{
			static ModEvents m;
			return &m;
		}
		BSEventNotifyControl ProcessEvent(const SKSE::ModCallbackEvent* a_event, BSTEventSource<SKSE::ModCallbackEvent>*) override
		{
			if (!a_event)
				return BSEventNotifyControl::kContinue;
			const std::string_view name = a_event->eventName.c_str() ? a_event->eventName.c_str() : "";
			const int v = a_event->numArg < -0.5f ? -1 : (a_event->numArg > 0.5f ? 1 : 0);
			if (name == "DeepFit_MegaMouth") {
				g_megaOverride = v;
				logs::info("DeepFit: mega mouth {} (mod event)", v < 0 ? "back to the setting" : (v ? "on" : "off"));
			} else if (name == "DeepFit_Gulp" || name == "DeepFit_Gag" || name == "DeepFit_Shape") {   // 0.4.0: from Anatomica, sender = him
				auto* who = a_event->sender ? a_event->sender->As<Actor>() : nullptr;
				if (who) {
					auto& r = g_receivers[who->GetFormID()];
					if (name != "DeepFit_Shape")
						r.extAt = g_clock;   // 0.4.0: Anatomica sends them - DeepFit's own wait
					if (name == "DeepFit_Gulp")
						r.gulpAt = g_clock;
					else if (name == "DeepFit_Gag") {
						r.gagAt = g_clock;
						r.gagK = std::clamp(a_event->numArg, 0.3f, 2.0f);
					} else {
						r.shape = static_cast<int>(a_event->numArg + 0.5f);
						r.shapeAt = g_clock;
						r.shapeFromAnatomica = true;
					}
				}
			} else if (name == "DeepFit_Mouth") {
				g_mouthOverride = v;
				logs::info("DeepFit: opening the mouth {} (mod event)", v < 0 ? "back to the setting" : (v ? "on" : "off"));
			}
			return BSEventNotifyControl::kContinue;
		}
	};

	// 0.4.4: sounds - a swallow, a gag, a breath after he pulls out. WAV / XWM files in Data\\Sound\\Fx\\DeepFit\\<gulp / gag /
	// gasp> (any number, one picked at random); with none for swallows, Anatomica's gulps are used
	struct SoundKind
	{
		std::vector<std::string> files;
		int                      last = -1;
		bool                     looked = false;
	};
	std::unordered_map<std::string, SoundKind> g_sounds;
	std::mutex                                 g_soundLock;   // (the menu counts the files too)

	SoundKind& lookSounds(const std::string& kind)
	{
		auto& k = g_sounds[kind];
		if (k.looked)
			return k;
		k.looked = true;
		auto scan = [&](const char* dirName) {
			std::error_code ec;
			const std::filesystem::path dir = std::filesystem::path("Data/Sound/Fx") / dirName / kind;
			for (auto it = std::filesystem::directory_iterator(dir, ec); !ec && it != std::filesystem::directory_iterator(); it.increment(ec)) {
				if (!it->is_regular_file(ec))
					continue;
				std::string ext = it->path().extension().string();
				for (auto& c : ext) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
				if (ext == ".wav" || ext == ".xwm")
					k.files.push_back(std::string("Sound\\Fx\\") + dirName + "\\" + kind + "\\" + it->path().filename().string());
			}
		};
		scan("DeepFit");
		if (k.files.empty() && kind == "gulp")
			scan("Anatomica");
		std::sort(k.files.begin(), k.files.end());
		logs::info("DeepFit: {} sound file(s) for \"{}\"", k.files.size(), kind);
		return k;
	}

	// (CommonLibSSE-NG 3.7 has neither of these two: the game's own functions, called directly)
	void idFromPath(BSResource::ID& a_id, const char* a_path)
	{
		using func_t = void (*)(BSResource::ID*, const char*);
		static REL::Relocation<func_t> func{ RELOCATION_ID(68635, 69979) };
		func(&a_id, a_path);
	}

	void soundHandleByFile(BSAudioManager* a_am, BSSoundHandle& a_handle, const BSResource::ID& a_file, std::uint32_t a_flags, std::uint32_t a_priority)
	{
		using func_t = void (*)(BSAudioManager*, BSSoundHandle&, const BSResource::ID&, std::uint32_t, std::uint32_t);
		static REL::Relocation<func_t> func{ RELOCATION_ID(66402, 67664) };
		func(a_am, a_handle, a_file, a_flags, a_priority);
	}

	void playSound(const char* kind, Actor* a, float volume)
	{
		auto* am = BSAudioManager::GetSingleton();
		if (!a || !am || volume <= 0.0f)
			return;
		std::string file;
		{
			std::lock_guard lk(g_soundLock);
			auto& k = lookSounds(kind);
			if (k.files.empty())
				return;
			int i = std::uniform_int_distribution<int>(0, static_cast<int>(k.files.size()) - 1)(g_rng);
			if (k.files.size() > 1 && i == k.last)
				i = (i + 1) % static_cast<int>(k.files.size());
			k.last = i;
			file = k.files[static_cast<std::size_t>(i)];
		}
		BSResource::ID id;
		idFromPath(id, file.c_str());
		BSSoundHandle h;
		soundHandleByFile(am, h, id, 0x1A, 0);
		if (!h.IsValid())
			return;
		auto* root = a->Get3D(false);
		if (auto* at = root ? root->GetObjectByName("NPC Head [Head]") : nullptr)
			h.SetObjectToFollow(at);
		else
			h.SetPosition(a->GetPosition());
		h.SetVolume(std::clamp(volume, 0.0f, 2.0f));
		h.SetFrequency(std::uniform_real_distribution<float>(0.94f, 1.06f)(g_rng));
		h.Play();
	}

	// each new swallow / gag / pull-out once (Anatomica's own swallows and gags are left to it unless asked)
	void soundsFor(Receiver& r, Actor* a)
	{
		const auto& cfg = cfgOf(r);
		auto own = [&](double at) { return std::fabs(at - r.extAt) > 1e-6; };
		if (r.gulpAt > r.gulpSounded) {
			r.gulpSounded = r.gulpAt;
			if (g_clock - r.gulpAt < 0.5 && (own(r.gulpAt) || !cfg.soundOwnOnly))
				playSound("gulp", a, cfg.soundGulp);
		}
		if (r.gagAt > r.gagSounded) {
			r.gagSounded = r.gagAt;
			if (g_clock - r.gagAt < 0.5 && (own(r.gagAt) || !cfg.soundOwnOnly))
				playSound("gag", a, cfg.soundGag);
		}
		if (r.leftAt > r.gaspSounded) {
			r.gaspSounded = r.leftAt;
			if (g_clock - r.leftAt < 0.5)
				playSound("gasp", a, cfg.soundGasp);
		}
	}

	// 0.4.2: what his face should show (eyes / brows), per actor; set by the frame task, put on in the animation update
	struct FaceWant
	{
		float want[8] = {};
	};
	struct FaceState
	{
		float want[8] = {}, base[8] = {}, sent[8] = { -1, -1, -1, -1, -1, -1, -1, -1 };
		bool  on = false, wanted = false;
		int   ph = -1;                                // 0.4.5: his jaw opened with a phoneme (a head without DeepFit's morphs)
		float phWant = 0.0f, phBase = 0.0f, phSent = -1.0f;
	};
	std::mutex                              g_faceLock;
	std::unordered_map<FormID, FaceState>   g_face;
	std::atomic<int>                        g_faceN{ 0 };

	void setFaceWant(FormID id, const FaceWant& fw)
	{
		bool any = false;
		for (float w : fw.want)
			any |= w > 0.01f;
		std::lock_guard lk(g_faceLock);
		auto it = g_face.find(id);
		if (it == g_face.end()) {
			if (!any)
				return;
			it = g_face.emplace(id, FaceState{}).first;
			g_faceN = static_cast<int>(g_face.size());
		}
		std::copy(std::begin(fw.want), std::end(fw.want), it->second.want);
		it->second.wanted = any;
	}

	// 0.4.5: his jaw by a phoneme (0-1), for a head whose profile names one (its own morphs are not there)
	void setJawWant(FormID id, int ph, float v)
	{
		std::lock_guard lk(g_faceLock);
		auto it = g_face.find(id);
		if (it == g_face.end()) {
			if (v <= 0.005f)
				return;
			it = g_face.emplace(id, FaceState{}).first;
			g_faceN = static_cast<int>(g_face.size());
		}
		it->second.ph = ph;
		it->second.phWant = v;
	}

	// right after the game poses him: over whatever OStim's expression / the blinking set (never below it)
	void applyFace(Actor* a)
	{
		if (g_faceN.load(std::memory_order_relaxed) == 0 || !a)
			return;
		std::lock_guard lk(g_faceLock);
		auto it = g_face.find(a->GetFormID());
		if (it == g_face.end())
			return;
		auto& f = it->second;
		auto* fg = a->GetFaceGenAnimationData();
		if (!fg || !fg->modifierKeyFrame.values || fg->modifierKeyFrame.count < 8)
			return;
		auto* vals = fg->modifierKeyFrame.values;
		for (std::uint32_t i = 0; i < 8; ++i) {
			if (i == 2 || i == 3)
				continue;
			const float cur = vals[i];
			if (std::fabs(cur - f.sent[i]) > 1e-3f)
				f.base[i] = cur;   // someone else set it since: that is what we go over
			const float out = f.wanted ? std::max(f.base[i], f.want[i]) : f.base[i];
			fg->modifierKeyFrame.SetValue(i, out);
			f.sent[i] = out;
		}
		if (f.ph >= 0 && fg->phenomeKeyFrame.values && static_cast<std::uint32_t>(f.ph) < fg->phenomeKeyFrame.count) {
			const float cur = fg->phenomeKeyFrame.values[f.ph];
			if (std::fabs(cur - f.phSent) > 1e-3f)
				f.phBase = cur;   // the lip sync / OStim's expression set it since
			// 0.4.7: while DeepFit holds his jaw, its own value alone - the scene's expression animates the same phoneme and
			// going over it made the jaw twitch
			const float out = f.phWant > 0.005f ? f.phWant : f.phBase;
			fg->phenomeKeyFrame.SetValue(f.ph, out);
			f.phSent = out;
			if (f.phWant <= 0.005f)
				f.ph = -1;   // given back
		}
		f.on = f.wanted || f.ph >= 0;
		if (!f.on) {   // given back: forget him
			g_face.erase(it);
			g_faceN = static_cast<int>(g_face.size());
		}
	}

	// 0.2.0: hand the mouth morphs to MFEE (its Papyrus function, on the game thread we run on) and the teeth modifier
	// to the face; only when a value has really changed, at most 20 times a second
	void sendMouth(Receiver& r, float delta)
	{
		auto& cfg = cfgOf(r);
		if (!DeepFit::MouthNow())
			return;
		auto act = r.actor.get();
		if (!act)
			return;
		{   // 0.4.5: a head profile that opens the jaw with a phoneme (no DeepFit morphs in that head)
			auto& hs = Heads::All();
			if (r.profile >= 0 && r.profile < static_cast<int>(hs.size()) && hs[r.profile].jawPhoneme >= 0)
				setJawWant(act->GetFormID(), hs[r.profile].jawPhoneme, std::clamp(r.morph[0] / 100.0f, 0.0f, 1.0f));
		}
		if (auto* fg = act->GetFaceGenAnimationData(); fg && cfg.teethOpen && (r.teeth || r.teethSent)) {   // 0.2.1: only while in use, and once to close
			fg->modifierKeyFrame.SetValue(12, r.teeth ? 1.0f : 0.0f);
			r.teethSent = r.teeth;
		}
		// 0.4.0: his face reacts - eyes close and brows lift the deeper it goes, eyes clench and brows knit in a gag.
		// 0.4.2: worked out here, put on his face in the game's animation update (setFace); written from here they were
		// overwritten by OStim's expressions / blinking in some frames and not others - the eyes and brows flickered
		{
			FaceWant fw;
			if (cfg.faceReact) {
				const double tk = g_clock - r.gagAt;
				const float  gag = (cfg.gagTime > 0.0f && tk >= 0.0 && tk < cfg.gagTime) ? std::sin(3.14159f * static_cast<float>(tk / cfg.gagTime)) * std::min(r.gagK, 1.0f) : 0.0f;
				const float  eyes = std::clamp(cfg.faceEyes * r.depthK + cfg.faceGag * gag, 0.0f, 1.0f);
				fw.want[0] = fw.want[1] = eyes;                                                       // blink left / right
				fw.want[4] = fw.want[5] = std::clamp(cfg.faceGag * gag * 0.8f, 0.0f, 1.0f);           // brows in
				fw.want[6] = fw.want[7] = std::clamp(cfg.faceBrows * r.depthK * (1.0f - gag), 0.0f, 1.0f);   // brows up
			}
			setFaceWant(act->GetFormID(), fw);
		}
		r.sendTimer -= delta;
		if (r.sendTimer > 0.0f)
			return;
		r.sendTimer = 0.05f;
		auto* vm = BSScript::Internal::VirtualMachine::GetSingleton();
		if (!vm)
			return;
		// 0.4.0: a swallow running down his throat, and a gag (the jaw drops a little, the throat tightens)
		float extra[kMorphs] = {};
		const double tg = g_clock - r.gulpAt, tk = g_clock - r.gagAt;
		if (cfg.gulpSize > 0.0f && tg >= 0.0 && tg < cfg.gulpTime) {
			const float pos = static_cast<float>(tg / cfg.gulpTime) * (kBands + 2) - 1.0f;
			for (int k = 0; k < kBands; ++k)
				extra[kBand0 + k] += 100.0f * cfg.gulpSize * std::exp(-std::pow((k - pos) / 1.1f, 2.0f));
		}
		if (cfg.gagTime > 0.0f && tk >= 0.0 && tk < cfg.gagTime) {
			const float env = std::sin(3.14159f * static_cast<float>(tk / cfg.gagTime)) * r.gagK;
			extra[0] += cfg.gagJaw * env;
			for (int k = 0; k < kBands; ++k)
				extra[kBand0 + k] -= r.morph[kBand0 + k] * std::min(1.0f, cfg.gagSqueeze * env);
		}
		for (int m = 0; m < kMorphs; ++m) {
			// (0.4.4: the swelling families are 3 units at 100 in the head - a third of the value, so they stay under MFEE's 100)
			const float raw = (r.morph[m] + extra[m]) / (throatScaled(m) ? kThroatScale : 1.0f);
			const int   v = std::max(0, static_cast<int>(std::lround(raw)));
			if (r.sent[m] >= 0 && std::abs(v - r.sent[m]) < 2 && !(v == 0 && r.sent[m] != 0))
				continue;
			r.sent[m] = v;
			BSTSmartPointer<BSScript::IStackCallbackFunctor> cb;
			auto* args = MakeFunctionArguments(static_cast<Actor*>(act.get()), BSFixedString(kMorphNames[m]), static_cast<std::int32_t>(v),
				static_cast<std::int32_t>(cfg.morphLerpMs));
			vm->DispatchStaticCall("MuFacialExpressionExtended", "SetExpressionByNameEx", args, cb);
			if (!r.toldMouth && cfg.debugLog) {
				r.toldMouth = true;
				logs::info("DeepFit: driving the mouth morphs through MFEE ({}...)", kMorphNames[m]);
			}
		}
		// 0.5.4: his beards over the lip bones, as far as each bone moved (out or in)
		if (r.beardBones && Config::get().beardFollow && Config::get().beardBones)
			for (int sp = 0; sp < r.nspecs() && sp < kMaxSpecs; ++sp) {
				if (!(r.beardBones & (1ull << sp)))
					continue;
				const float cur = r.bones[sp].valid ? r.bones[sp].cur : 0.0f;
				for (int in = 0; in < 2; ++in) {
					const float raw = (in ? -cur : cur) / kBeardBoneUnits * 100.0f;
					const int   v = std::clamp(static_cast<int>(std::lround(raw)), 0, 100);
					int&        last = r.beardSent[sp * 2 + in];
					if (last >= 0 && std::abs(v - last) < 2 && !(v == 0 && last != 0))
						continue;
					last = v;
					BSTSmartPointer<BSScript::IStackCallbackFunctor> cb;
					auto* args = MakeFunctionArguments(static_cast<Actor*>(act.get()), BSFixedString(beardBoneName(sp, in == 1).c_str()), static_cast<std::int32_t>(v),
						static_cast<std::int32_t>(cfg.morphLerpMs));
					vm->DispatchStaticCall("MuFacialExpressionExtended", "SetExpressionByNameEx", args, cb);
				}
			}
	}

	// 0.4.0: the teeth moved a little down / back. Their weights on his head bone are hung on a node of DeepFit's own under
	// it ("DeepFit Teeth"), so they follow his head exactly and only that node is moved. The game rebuilding the face puts
	// them back on the head bone: they are checked every second and hung on it again. Mode 1 while the mouth is full,
	// mode 2 always (only the actors DeepFit fits, and only when chosen).
	// 0.4.6: his teeth and mouth parts (TeethParts) hidden while he is in a scene - on a beast head the jaw drops past them
	// (they follow only its own jaw shape) - and shown again after
	// 0.5.4: DeepFit's rigged mouths - their bones 2-4 hang on nodes that copy how far his three lower jaw bones moved (so
	// the gums and floor of his mouth go down and fold with the jaw, no gap under the lips), bone 5 on one the shaft pushes
	// down (the tongue). The nodes sit at his head bone, as the mouth's own bones do, so nothing moves while DeepFit is idle
	void rigMouth(Receiver& r, Actor* act, float delta)
	{
		auto* headNode = r.head ? r.head->AsNode() : nullptr;
		if (!headNode)
			return;
		for (auto& n : r.mouthNodes)
			if (!n || n->parent != headNode) {
				n = NiPointer<NiNode>(NiNode::Create(0));
				n->name = "DF Mouth Rig";
				n->local = NiTransform();
				headNode->AttachChild(n.get(), true);
			}
		// where each lower jaw bone is against its rest, in his head bone's space (its parents only move, never turn)
		static const char* kJaw[3] = { "Lower Jaw Front", "Lower Jaw Mid", "Lower Jaw Back" };
		for (int k = 0; k < 3; ++k) {
			NiPoint3 d;
			for (int i = 0; i < r.nspecs(); ++i)
				if (r.specs[i].name == kJaw[k] && r.bones[i].valid && r.bones[i].node) {
					d = r.bones[i].node->local.translate - r.bones[i].restLocal;
					break;
				}
			r.mouthNodes[k]->local = NiTransform();
			r.mouthNodes[k]->local.translate = d;
		}
		// 0.5.5: the edges of the palate and the floor of his mouth follow his upper and lower lips (they were left behind
		// when the lip bones pushed the lips out - a see-through gap between the lips and the inside of the mouth)
		{
			static const char* kUp[] = { "Upper Lip Center", "Upper Lip Left", "Upper Lip Right", "Muzzle Upper Left", "Muzzle Upper Right" };
			static const char* kLo[] = { "Bottom Lip Center", "Bottom Lip Left", "Bottom Lip Right", "Muzzle Lower Left", "Muzzle Lower Right", "Muzzle Jaw Left", "Muzzle Jaw Right" };
			auto avg = [&](const char* const* names, std::size_t n) {
				NiPoint3 sum;
				int      k = 0;
				for (std::size_t q = 0; q < n; ++q)
					for (int i = 0; i < r.nspecs(); ++i)
						if (r.specs[i].name == names[q] && r.bones[i].valid && r.bones[i].node) {
							sum += r.bones[i].node->local.translate - r.bones[i].restLocal;
							++k;
							break;
						}
				return k ? sum * (1.0f / static_cast<float>(k)) : NiPoint3();
			};
			r.mouthNodes[4]->local = NiTransform();
			r.mouthNodes[4]->local.translate = avg(kUp, std::size(kUp));
			r.mouthNodes[5]->local = NiTransform();
			r.mouthNodes[5]->local.translate = avg(kLo, std::size(kLo));
			for (int q = 0; q < kMouthRimN; ++q) {   // 0.5.6: each of his lip bones on its own
				r.mouthNodes[6 + q]->local = NiTransform();
				r.mouthNodes[6 + q]->local.translate = avg(&kMouthRim[q], 1);
			}
		}
		// the tongue: on the shaft's underside (down, or up to it), and - 0.5.5 - licking forward along it now and then
		r.tongueCur += (r.tongueWant - r.tongueCur) * (1.0f - std::exp(-delta * 10.0f));
		{
			const auto& cfgT = cfgOf(r);
			const float want = r.tongueLicks && cfgT.tongueLick > 0.0f ?
			                       cfgT.tongueLick * 0.5f * (1.0f + std::sin(static_cast<float>(g_clock) * 2.0f * 3.14159f * cfgT.tongueLickRate)) : 0.0f;
			r.tongueFwd += (want - r.tongueFwd) * (1.0f - std::exp(-delta * 8.0f));
			const NiTransform& H = r.head->world;
			const float        hs = H.scale > 1e-6f ? H.scale : 1.0f;
			r.mouthNodes[3]->local = NiTransform();
			// 0.5.9: on top of where the middle of his lower jaw went (the tongue lies on the floor of his mouth - the
			// tongue alone was pushed while the floor went with the jaw, and they went through each other)
			r.mouthNodes[3]->local.translate = r.mouthNodes[1]->local.translate + (H.rotate.Transpose() * (r.fz * -r.tongueCur + r.fy * r.tongueFwd)) * (1.0f / hs);
		}
		for (auto& n : r.mouthNodes)
			n->world = r.head->world * n->local;
		// 0.5.9: the throat: each throat bone goes to the middle of the shaft where it passes that part of the throat, and
		// widens round it; back to rest when nothing is there
		for (auto& n : r.throatNodes)
			if (!n || n->parent != headNode) {
				n = NiPointer<NiNode>(NiNode::Create(0));
				n->name = "DF Throat Rig";
				n->local = NiTransform();
				headNode->AttachChild(n.get(), true);
			}
		if (r.throatSet) {
			const NiTransform& H = r.head->world;
			const float        hsc = H.scale > 1e-6f ? H.scale : 1.0f;
			std::vector<NiPoint3> pts;
			pts.reserve(r.shaftWorld.size());
			for (auto& p : r.shaftWorld)
				pts.push_back(H.rotate.Transpose() * (p - H.translate) * (1.0f / hsc));
			const float k = 1.0f - std::exp(-delta * 10.0f);
			for (int q = 0; q < kThroatN; ++q) {
				const NiPoint3 c = r.throatC[q];
				NiPoint3       t = r.throatC[std::min(q + 1, kThroatN - 1)] - r.throatC[std::max(q - 1, 0)];
				t = len(t) > 1e-4f ? norm(t) : NiPoint3(0, -1, 0);
				NiPoint3 m;
				int      cnt = 0;
				for (auto& p : pts)
					if (std::fabs(dot(p - c, t)) < 0.8f && len(p - c) < 12.0f) {
						m += p;
						++cnt;
					}
				NiPoint3 wantOff;
				float    wantK = 1.0f;
				if (cnt >= 12) {
					m = m * (1.0f / static_cast<float>(cnt));
					float rad = 0.0f;
					for (auto& p : pts)
						if (std::fabs(dot(p - c, t)) < 0.8f && len(p - c) < 12.0f) {
							NiPoint3 d = p - m;
							d = d - t * dot(d, t);
							rad = std::max(rad, len(d));
						}
					wantOff = (m - c) - t * dot(m - c, t);
					if (len(wantOff) > 8.0f)
						wantOff = norm(wantOff) * 8.0f;
					wantK = std::clamp((rad + 0.2f) / std::max(r.throatR[q], 0.1f), 1.0f, 3.0f);
				}
				r.throatOff[q] += (wantOff - r.throatOff[q]) * k;
				r.throatK[q] += (wantK - r.throatK[q]) * k;
				auto& n = r.throatNodes[q];
				n->local = NiTransform();
				n->local.translate = c + r.throatOff[q];
				n->local.scale = r.throatR[q] * r.throatK[q];
				n->world = H * n->local;
			}
		}
		// hang the rigged mouths' bones on them (again after the face is rebuilt)
		r.mouthScan -= delta;
		if (r.mouthScan > 0.0f)
			return;
		r.mouthScan = 1.0f;
		auto* root = act ? act->Get3D(false) : nullptr;
		if (!root)
			return;
		traverse(root, [&](NiAVObject* o) {
			auto* geo = o ? o->AsGeometry() : nullptr;
			auto* si = geo ? geo->GetGeometryRuntimeData().skinInstance.get() : nullptr;
			const std::uint32_t nb = si && si->skinData ? si->skinData->bones : 0;
			if (!si || !si->skinData || !si->bones || !si->boneWorldTransforms || !rigMouthBones(nb))
				return;
			auto onHead = [&](NiAVObject* b) {
				return b && (b == r.head.get() || (b->name.c_str() && std::strcmp(b->name.c_str(), "NPC Head [Head]") == 0));
			};
			if (!onHead(si->bones[0]))
				return;
			for (std::uint32_t i = 1; i < nb; ++i) {
				// 0.5.9: its teeth: the upper ones (bone 1) on his head - the snout's own morph moves them with it; on the
				// upper lip (0.5.8) they followed the lip pressing down onto the shaft and hung in it. A 25-bone mouth's
				// lower teeth (bone 24) go with the front of his lower jaw. While hidden for a scene both stay on the
				// hiding node
				const bool teethSlot = i == 1 || static_cast<int>(i) == kTeethLowerSlot;
				if (teethSlot && r.teethHideNode && si->bones[i] == r.teethHideNode.get())
					continue;
				const int ti = static_cast<int>(i) - kThroatSlot0;
				if (ti >= 0 && ti < kThroatN && !r.throatSet) {   // its throat bones' rest (from its skin: centre, 1/radius)
					const auto& s0 = si->skinData->boneData[0].skinToBone;
					for (int q = 0; q < kThroatN; ++q) {
						const auto& sq = si->skinData->boneData[kThroatSlot0 + q].skinToBone;
						const float rq = sq.scale > 1e-6f ? 1.0f / sq.scale : 1.0f;
						r.throatR[q] = rq;
						r.throatC[q] = s0.translate - sq.translate * rq;
						r.throatOff[q] = NiPoint3();
						r.throatK[q] = 1.0f;
					}
					r.throatSet = true;
					if (Config::get().debugLog)
						logs::info("DeepFit: {} - its throat follows the shaft ({} bones, from {:.1f}/{:.1f}/{:.1f})", geo->name.c_str() ? geo->name.c_str() : "?", kThroatN,
							r.throatC[0].x, r.throatC[0].y, r.throatC[0].z);
				}
				NiNode* want = (ti >= 0 && ti < kThroatN) ? r.throatNodes[ti].get() :
				               i == 1 ? headNode : static_cast<int>(i) == kTeethLowerSlot ? r.mouthNodes[0].get() : r.mouthNodes[i - 2].get();
				if (si->bones[i] == want)
					continue;
				r.mouthSlots.push_back({ NiPointer<BSGeometry>(geo), i, NiPointer<NiAVObject>(si->bones[i]) });
				si->bones[i] = want;
				si->boneWorldTransforms[i] = &want->world;
				if (Config::get().debugLog && i == 2)
					logs::info("DeepFit: {} - its lower gums follow his jaw bones and its tongue the shaft", geo->name.c_str() ? geo->name.c_str() : "?");
			}
		});
	}

	// 0.5.5: a head with a skeleton jaw bone: the bone turned open as far as DeepFit's jaw opening says (at 100: its
	// head profile's JawBoneDeg), on top of where the animation put it this frame; which way opens is found once (the
	// way that takes the front of the jaw down)
	void turnJaw(Receiver& r)
	{
		auto* jaw = r.jawBone.get();
		if (!jaw || !jaw->parent || !r.head)
			return;
		// the animation's own pose this frame (it may not set the jaw every frame - then ours is still there)
		if (!r.jawSetValid || !(jaw->local.rotate == r.jawSet))
			r.jawBase = jaw->local.rotate;
		const float open = std::clamp(r.morph[0] / 100.0f, 0.0f, 1.5f) * r.jawBoneDeg * 0.0174533f;
		auto turned = [&](float a) {
			// about his face's right axis, in the jaw bone's parent space
			const NiMatrix3& PR = jaw->parent->world.rotate;
			NiPoint3         ax = PR.Transpose() * r.fx;
			ax = norm(ax);
			const float c = std::cos(a), s = std::sin(a), t = 1.0f - c;
			NiMatrix3   R;
			R.entry[0][0] = t * ax.x * ax.x + c;        R.entry[0][1] = t * ax.x * ax.y - s * ax.z; R.entry[0][2] = t * ax.x * ax.z + s * ax.y;
			R.entry[1][0] = t * ax.x * ax.y + s * ax.z; R.entry[1][1] = t * ax.y * ax.y + c;        R.entry[1][2] = t * ax.y * ax.z - s * ax.x;
			R.entry[2][0] = t * ax.x * ax.z - s * ax.y; R.entry[2][1] = t * ax.y * ax.z + s * ax.x; R.entry[2][2] = t * ax.z * ax.z + c;
			return R * r.jawBase;
		};
		if (r.jawSign == 0.0f) {   // which way is open: the jaw's front (towards his lips) goes down
			const NiPoint3 lips = r.bones[kBottomCenter].valid ? r.bones[kBottomCenter].parent->world * r.bones[kBottomCenter].restLocal : r.head->world.translate;
			const NiPoint3 local = jaw->world.rotate.Transpose() * (lips - jaw->world.translate) * (1.0f / std::max(jaw->world.scale, 1e-6f));
			const NiTransform& P = jaw->parent->world;
			auto frontZ = [&](float a) {
				NiTransform t = jaw->local;
				t.rotate = turned(a);
				const NiTransform w = P * t;
				return dot(w * local, r.fz);
			};
			r.jawSign = frontZ(0.3f) < frontZ(-0.3f) ? 1.0f : -1.0f;
		}
		jaw->local.rotate = turned(open * r.jawSign);
		r.jawSet = jaw->local.rotate;
		r.jawSetValid = true;
		NiUpdateData ud{};
		jaw->Update(ud);
	}

	void hideTeeth(Receiver& r, Actor* act, float delta)
	{
		const auto& cfg = cfgOf(r);
		const bool  hide = cfg.teethHide && r.inScene;
		if (!hide) {
			for (auto& o : r.teethHidden)
				if (o)
					o->SetAppCulled(false);
			r.teethHidden.clear();
			for (auto& h : r.teethSlots) {   // 0.5.2: the teeth back on the bone they hung on
				auto* si = h.geo ? h.geo->GetGeometryRuntimeData().skinInstance.get() : nullptr;
				if (si && si->bones && si->skinData && h.idx < si->skinData->bones && si->bones[h.idx] == r.teethHideNode.get() && h.orig) {
					si->bones[h.idx] = h.orig.get();
					si->boneWorldTransforms[h.idx] = &h.orig->world;
				}
			}
			r.teethSlots.clear();
			for (auto& b : r.teethBones)
				if (b) {
					b->local.scale = 1.0f;
					if (b->parent)
						b->world = b->parent->world * b->local;
					else if (r.head)
						b->world = r.head->world;
				}
			r.teethBones.clear();
			return;
		}
		for (auto& b : r.teethBones)   // 0.4.7: shrunk to nothing every frame (the teeth only)
			if (b) {
				b->local.scale = 0.001f;
				if (b->parent)   // (as the mouth bones: its world set here too - the game may not update it)
					b->world = b->parent->world * b->local;
				else if (r.head) {   // not in the scene graph: at his head bone, shrunk
					b->world = r.head->world;
					b->world.scale *= 0.001f;
				}
			}
		r.hideScan -= delta;
		if (r.hideScan > 0.0f && (!r.teethHidden.empty() || !r.teethBones.empty()))
			return;
		r.hideScan = 1.0f;   // again every second (the face is sometimes rebuilt)
		auto* root = act ? act->Get3D(false) : nullptr;
		if (!root)
			return;
		// 0.4.7: a mouth whose teeth hang on their own bone (DeepFit's Argonian mouth): only that bone is shrunk - his
		// tongue and gums stay
		r.teethBones.clear();
		traverse(root, [&](NiAVObject* o) {
			if (!o)
				return;
			auto add = [&](NiAVObject* b) {
				if (b && std::find_if(r.teethBones.begin(), r.teethBones.end(), [&](auto& x) { return x.get() == b; }) == r.teethBones.end()) {
					b->local.scale = 0.001f;
					r.teethBones.emplace_back(b);
				}
			};
			if (!o->AsGeometry() && o->name.c_str() && std::strcmp(o->name.c_str(), "DF Teeth Only") == 0)
				add(o);
			if (auto* geo = o->AsGeometry()) {   // (and through the mouth's own skin)
				auto* si = geo->GetGeometryRuntimeData().skinInstance.get();
				if (si && si->bones && si->skinData)
					for (std::uint32_t i = 0; i < si->skinData->bones; ++i)
						if (si->bones[i] && si->bones[i]->name.c_str() && std::strcmp(si->bones[i]->name.c_str(), "DF Teeth Only") == 0)
							add(si->bones[i]);
			}
		});
		// 0.5.2: the game hangs a head part's bones on his skeleton by name when the face is built; "DF Teeth Only" is not
		// in the skeleton, so it was hung on his head bone and could not be found by name - the whole mouth was hidden
		// (tongue and all). DeepFit's mouths put the teeth on the mouth's second bone: a mouth part (TeethParts) whose later
		// bone is his head bone again is that one - it is hung on a node of DeepFit's own, shrunk to nothing
		if (r.teethBones.empty()) {
			std::vector<std::string> tp;
			{
				std::string cur;
				for (char c : Config::get().teethParts + ",") {
					if (c == ',') {
						while (!cur.empty() && cur.back() == ' ') cur.pop_back();
						if (!cur.empty()) tp.push_back(cur);
						cur.clear();
					} else if (!(cur.empty() && c == ' '))
						cur += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
				}
			}
			auto* headNode = r.head ? r.head->AsNode() : nullptr;
			if (headNode && (!r.teethHideNode || r.teethHideNode->parent != headNode)) {
				r.teethHideNode = NiPointer<NiNode>(NiNode::Create(0));
				r.teethHideNode->name = "DF Teeth Hidden";
				r.teethHideNode->local = NiTransform();
				r.teethHideNode->local.scale = 0.001f;
				r.teethHideNode->GetFlags().set(NiAVObject::Flag::kSelectiveUpdate, NiAVObject::Flag::kSelectiveUpdateTransforms,
					NiAVObject::Flag::kSelectiveUpdateController);
				headNode->AttachChild(r.teethHideNode.get(), true);
			}
			if (r.teethHideNode) {
				r.teethHideNode->local.scale = 0.001f;
				r.teethHideNode->world = r.head->world * r.teethHideNode->local;
				traverse(root, [&](NiAVObject* o) {
					auto* geo = o ? o->AsGeometry() : nullptr;
					auto* si = geo ? geo->GetGeometryRuntimeData().skinInstance.get() : nullptr;
					if (!si || !si->skinData || !si->bones || !si->boneWorldTransforms || si->skinData->bones < 2)
						return;
					std::string lo = geo->name.c_str() ? geo->name.c_str() : "";
					for (auto& c : lo) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
					bool match = false;
					for (auto& pt : tp) match |= lo.find(pt) != std::string::npos;
					if (!match)
						return;
					auto onHead = [&](NiAVObject* b) {
						return b && (b == r.head.get() || b == r.teethNode.get() || (b->name.c_str() && std::strcmp(b->name.c_str(), "NPC Head [Head]") == 0));
					};
					// 0.5.3: DeepFit's mouths have exactly two bones (his head, then the teeth): the teeth's one is re-hung
					// whatever the game made of it (on the Argonian's it was neither his head bone nor found by name - the
					// whole mouth, tongue and all, was hidden again)
					const bool rig = rigMouthBones(si->skinData->bones);   // 0.5.4: a rigged mouth - only its teeth slot (1) here
					const bool ours = si->skinData->bones == 2 || rig;
					if (cfg.debugLog) {
						std::string names;
						for (std::uint32_t i = 0; i < si->skinData->bones; ++i)
							names += std::string(i ? ", " : "") + (si->bones[i] ? (si->bones[i]->name.c_str() ? si->bones[i]->name.c_str() : "(unnamed)") : "(none)");
						logs::info("DeepFit: {} - its bones: {}", geo->name.c_str() ? geo->name.c_str() : "?", names);
					}
					if (!ours && !onHead(si->bones[0]))
						return;
					for (std::uint32_t i = 1; i < si->skinData->bones; ++i) {
						if (si->bones[i] == r.teethHideNode.get())
							continue;   // ours already
						if (rig && i != 1 && static_cast<int>(i) != kTeethLowerSlot)
							continue;
						if (!ours && !onHead(si->bones[i]))
							continue;
						r.teethSlots.push_back({ NiPointer<BSGeometry>(geo), i, NiPointer<NiAVObject>(si->bones[i]) });
						si->bones[i] = r.teethHideNode.get();
						si->boneWorldTransforms[i] = &r.teethHideNode->world;
						if (cfg.debugLog)
							logs::info("DeepFit: {} - its teeth (bone {}) hidden for the scene - the tongue stays", geo->name.c_str() ? geo->name.c_str() : "?", i);
					}
				});
				for (auto& h : r.teethSlots)
					if (h.geo)
						r.teethBones.emplace_back(r.teethHideNode.get());
				if (!r.teethSlots.empty() && r.teethBones.size() > 1)
					r.teethBones.resize(1);
			}
		}
		if (!r.teethBones.empty()) {
			if (cfg.debugLog && r.teethHidden.empty())
				logs::info("DeepFit: his teeth hidden for the scene (their own bone - the tongue stays)");
			for (auto& o : r.teethHidden)   // (a part hidden before the bone was found)
				if (o)
					o->SetAppCulled(false);
			r.teethHidden.clear();
			return;
		}
		std::vector<std::string> parts;
		{
			std::string cur;
			for (char c : Config::get().teethParts + ",") {
				if (c == ',') {
					while (!cur.empty() && cur.back() == ' ') cur.pop_back();
					if (!cur.empty()) parts.push_back(cur);
					cur.clear();
				} else if (!(cur.empty() && c == ' '))
					cur += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
			}
		}
		traverse(root, [&](NiAVObject* o) {
			auto* geo = o ? o->AsGeometry() : nullptr;
			if (!geo || geo->GetAppCulled())
				return;
			std::string lo = geo->name.c_str() ? geo->name.c_str() : "";
			for (auto& c : lo) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
			bool match = false;
			for (auto& pt : parts) match |= lo.find(pt) != std::string::npos;
			if (!match)
				return;
			geo->SetAppCulled(true);
			r.teethHidden.emplace_back(geo);
			if (cfg.debugLog)
				logs::info("DeepFit: {} hidden for the scene", geo->name.c_str() ? geo->name.c_str() : "?");
		});
	}

	void updateTeeth(Receiver& r, Actor* act, float delta)
	{
		auto& cfg = cfgOf(r);
		// 0.4.2: only in a scene - 1 while his mouth is full, 2 while a partner's schlong is near him; otherwise they go
		// back to his head bone exactly as the game has them
		const bool wantOn = (cfg.teethMove == 2 && r.inScene) || (cfg.teethMove == 1 && r.full);
		const float to = wantOn ? 1.0f : 0.0f;
		r.teethK += (to - r.teethK) * (1.0f - std::exp(-delta * 6.0f));
		if (std::fabs(r.teethK - to) < 1e-3f)
			r.teethK = to;
		const bool moving = cfg.teethMove > 0 && (std::fabs(cfg.teethDown) > 1e-3f || std::fabs(cfg.teethBack) > 1e-3f);
		if (!moving || (r.teethK == 0.0f && !wantOn)) {
			if (!r.teethHooks.empty()) {   // not in use: back on his head bone, exactly where the game puts them
				for (auto& h : r.teethHooks) {
					auto* si = h.geo ? h.geo->GetGeometryRuntimeData().skinInstance.get() : nullptr;
					if (si && si->bones && h.idx < si->skinData->bones && si->bones[h.idx] == r.teethNode.get() && h.orig) {
						si->bones[h.idx] = h.orig.get();
						si->boneWorldTransforms[h.idx] = &h.orig->world;
					}
				}
				r.teethHooks.clear();
			}
			if (r.teethNode) {
				r.teethNode->local.translate = NiPoint3(0, 0, 0);
				if (r.teethNode->parent)   // 0.4.2: it follows his head again at once (not left where it last was)
					r.teethNode->world = r.teethNode->parent->world * r.teethNode->local;
			}
			return;
		}
		auto* root = act->Get3D(false);
		if (!root || !r.head)
			return;
		auto* headNode = r.head->AsNode();
		if (!headNode)
			return;
		if (!r.teethNode || r.teethNode->parent != headNode) {
			r.teethHooks.clear();
			r.teethNode = NiPointer<NiNode>(NiNode::Create(0));
			r.teethNode->name = "DeepFit Teeth";
			r.teethNode->local = NiTransform();
			// 0.4.2: the game updates it with his skeleton every frame (a node made without these was only ever moved
			// by DeepFit - between DeepFit's updates it stayed where it was, and his teeth hung outside his head)
			r.teethNode->GetFlags().set(NiAVObject::Flag::kSelectiveUpdate, NiAVObject::Flag::kSelectiveUpdateTransforms,
				NiAVObject::Flag::kSelectiveUpdateController);
			headNode->AttachChild(r.teethNode.get(), true);
			r.teethScan = 0.0f;
		}
		r.teethScan -= delta;
		if (r.teethScan <= 0.0f) {
			r.teethScan = 1.0f;
			std::vector<std::string> parts;
			{
				std::string cur;
				for (char c : Config::get().teethParts + ",") {
					if (c == ',') {
						while (!cur.empty() && cur.back() == ' ') cur.pop_back();
						if (!cur.empty()) parts.push_back(cur);
						cur.clear();
					} else if (!(cur.empty() && c == ' '))
						cur += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
				}
			}
			std::string seen;
			traverse(root, [&](NiAVObject* o) {
				auto* geo = o ? o->AsGeometry() : nullptr;
				if (!geo)
					return;
				auto* si = geo->GetGeometryRuntimeData().skinInstance.get();
				if (!si || !si->skinData || !si->bones || !si->boneWorldTransforms)
					return;
				std::string nm = geo->name.c_str() ? geo->name.c_str() : "";
				std::string lo = nm;
				for (auto& c : lo) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
				bool match = false;
				for (auto& pt : parts) match |= lo.find(pt) != std::string::npos;
				const std::uint32_t nb = si->skinData->bones;
				for (std::uint32_t i = 0; i < nb; ++i) {
					auto* b = si->bones[i];
					if (!match || !b)
						continue;
					// 0.5.8: a rigged mouth's head slot is the roof of his mouth and its edge on his lips - it stays on his
					// head. Hung on the teeth node it went down and back with "Teeth down / back" and the palate came away
					// from his upper lip (the gap under the upper lip); its teeth have their own slot
					if ((i <= 1 || static_cast<int>(i) == kTeethLowerSlot) && rigMouthBones(nb))
						continue;   // (its teeth: rigMouth)
					if (b == r.teethNode.get())
						continue;   // still ours
					const std::string_view bn(b->name.c_str() ? b->name.c_str() : "");
					const bool stale = bn == "DeepFit Teeth";   // 0.4.2: hung on an older node of ours (his face found again since)
					if (!stale && b != r.head.get() && bn != "NPC Head [Head]")
						continue;   // only what hangs on his head bone
					r.teethHooks.push_back({ NiPointer<BSGeometry>(geo), i, NiPointer<NiAVObject>(stale ? r.head.get() : b) });
					si->bones[i] = r.teethNode.get();
					si->boneWorldTransforms[i] = &r.teethNode->world;
				}
				if (cfg.debugLog && !r.teethTold && si->skinData->bones > 0)
					seen += (seen.empty() ? "" : ", ") + nm + (match ? " (teeth)" : "");
			});
			if (cfg.debugLog && !r.teethTold) {
				r.teethTold = true;
				logs::info("DeepFit: {:08X} skinned parts: {} - {} teeth weights moved to DeepFit Teeth", act->GetFormID(), seen, r.teethHooks.size());
			}
		}
		// down and back in his face's own axes, as his head bone's local offset
		const NiTransform& H = r.head->world;
		const NiPoint3 fy = r.axesOk ? r.fy : norm(H.rotate * NiPoint3(0, 1, 0)), fz = r.axesOk ? r.fz : norm(H.rotate * NiPoint3(0, 0, 1));
		const NiPoint3 offWorld = (fz * -cfg.teethDown + fy * -cfg.teethBack) * r.teethK;
		NiPoint3 off = H.rotate.Transpose() * offWorld;
		if (H.scale > 1e-6f)
			off = off * (1.0f / H.scale);
		r.teethNode->local.translate = off;
		r.teethNode->world = H * r.teethNode->local;
	}

	// 0.4.0: TNG's sizes, asked through its Papyrus functions (TNG_PapyrusUtil): the player's size category, and the scale
	// of each of the five categories as set in TNG's MCM
	std::mutex            g_tngLock;
	DeepFit::TngSizes     g_tng;
	std::atomic<bool>     g_tngAsk{ true };

	class PapyrusResult : public BSScript::IStackCallbackFunctor
	{
	public:
		explicit PapyrusResult(std::function<void(const BSScript::Variable&)> a_f) : f(std::move(a_f)) {}
		void operator()(BSScript::Variable a_result) override { if (f) f(a_result); }
		void SetObject(const BSTSmartPointer<BSScript::Object>&) override {}
		std::function<void(const BSScript::Variable&)> f;
	};

	void askTng()
	{
		auto* vm = BSScript::Internal::VirtualMachine::GetSingleton();
		auto* pc = PlayerCharacter::GetSingleton();
		if (!vm || !pc)
			return;
		{
			BSTSmartPointer<BSScript::IStackCallbackFunctor> cb(new PapyrusResult([](const BSScript::Variable& v) {
				if (!v.IsInt())
					return;
				std::lock_guard lk(g_tngLock);
				const int s = v.GetSInt();
				g_tng.mine = (s >= 0 && s <= 4) ? s : -1;
				g_tng.fromTng = true;
			}));
			auto* args = MakeFunctionArguments(static_cast<Actor*>(pc));
			vm->DispatchStaticCall("TNG_PapyrusUtil", "GetActorSize", args, cb);
		}
		for (int i = 0; i < 5; ++i) {
			BSTSmartPointer<BSScript::IStackCallbackFunctor> cb(new PapyrusResult([i](const BSScript::Variable& v) {
				if (!v.IsFloat())
					return;
				const float k = v.GetFloat();
				if (k > 0.05f && k < 5.0f) {
					std::lock_guard lk(g_tngLock);
					g_tng.scale[i] = k;
				}
			}));
			auto* args = MakeFunctionArguments(static_cast<std::int32_t>(i));
			vm->DispatchStaticCall("TNG_PapyrusUtil", "GetFloatValue", args, cb);
		}
	}

	// 0.4.0: the test schlong made visible - a plain round one, built from short pieces of DeepFit's own mesh (Data/meshes/
	// DeepFit\TestShaft.nif and TestShaftTip.nif) hung on the player's head, along his mouth
	NiPointer<NiNode>  g_testRoot, g_testSeg, g_testTip;
	std::vector<NiPointer<NiAVObject>> g_testPieces;
	NiPointer<NiAVObject> g_testTipPiece;
	bool               g_testLoadTried = false;

	void hideTestShaft()
	{
		if (g_testRoot && g_testRoot->parent)
			g_testRoot->parent->DetachChild(g_testRoot.get());
	}

	void updateTestShaft(Receiver& r)
	{
		const auto& pv = g_preview;
		if (!pv.on || !pv.show || !r.head) {
			hideTestShaft();
			return;
		}
		if (!g_testLoadTried) {
			g_testLoadTried = true;
			// 0.4.2: tried with and without "meshes\\" in front (the model database wants the full path), and the error said
			BSModelDB::DBTraits::ArgsType args{};
			auto load = [&](const char* name, NiPointer<NiNode>& out) {
				int e1 = -1, e2 = -1;
				for (const char* pre : { "meshes\\", "Meshes\\", "" }) {
					const std::string path = std::string(pre) + name;
					out.reset();
					const auto e = BSModelDB::Demand(path.c_str(), out, args);
					if (e == BSResource::ErrorCode::kNone && out)
						return true;
					(e1 < 0 ? e1 : e2) = static_cast<int>(e);
				}
				std::error_code ec;
				const bool      seen = std::filesystem::exists(std::string("Data/meshes/") + name, ec);
				logs::warn("DeepFit: the test schlong's mesh meshes\\{} did not load (errors {} / {}; 1 = the game cannot find the file - in MO2, refresh (F5) after files were added while it was open). The file is {}seen in Data\\meshes.", name, e1, e2, seen ? "" : "NOT ");
				return false;
			};
			NiPointer<NiNode> a, b;
			if (load("DeepFit\\TestShaft.nif", a) && load("DeepFit\\TestShaftTip.nif", b)) {
				g_testSeg = a;
				g_testTip = b;
				logs::info("DeepFit: the test schlong's meshes loaded");
			}
		}
		if (!g_testSeg || !g_testTip)
			return;
		auto* head = r.head->AsNode();
		if (!head)
			return;
		if (!g_testRoot) {
			g_testRoot = NiPointer<NiNode>(NiNode::Create(0));
			g_testRoot->name = "DeepFit TestShaft";
			g_testRoot->GetFlags().set(NiAVObject::Flag::kSelectiveUpdate, NiAVObject::Flag::kSelectiveUpdateTransforms,
				NiAVObject::Flag::kSelectiveUpdateController);   // 0.4.2: moves with his head every frame
			if (auto* t = static_cast<NiAVObject*>(g_testTip->Clone())) {
				g_testTipPiece = NiPointer<NiAVObject>(t);
				g_testRoot->AttachChild(t, true);
			}
		}
		if (g_testRoot->parent != head) {
			hideTestShaft();
			head->AttachChild(g_testRoot.get(), true);
		}
		// its frame: x his left, y into his mouth, z up, from the middle of his lips
		// (0.4.1: placed and angled as the test page says - the same as the points the lips fit)
		const NiPoint3 o0 = previewPlace(NiPoint3(0, 0, 0));
		auto toWorld = [&](const NiPoint3& f) { const NiPoint3 g = previewPlace(f) - o0; return r.fx * g.x + r.fy * g.y + r.fz * g.z; };
		const NiPoint3 ax = toWorld(NiPoint3(-1, 0, 0)), ay = toWorld(NiPoint3(0, -1, 0)), az = toWorld(NiPoint3(0, 0, 1));
		const NiPoint3 origin = r.mouthC + r.fx * o0.x + r.fy * o0.y + r.fz * o0.z;
		NiMatrix3      W;
		for (int i = 0; i < 3; ++i) {
			const float a[3] = { ax.x, ax.y, ax.z }, b[3] = { ay.x, ay.y, ay.z }, c[3] = { az.x, az.y, az.z };
			W.entry[i][0] = a[i];
			W.entry[i][1] = b[i];
			W.entry[i][2] = c[i];
		}
		const NiTransform& H = head->world;
		const float        inv = H.scale > 1e-6f ? 1.0f / H.scale : 1.0f;
		g_testRoot->local.rotate = H.rotate.Transpose() * W;
		g_testRoot->local.translate = (H.rotate.Transpose() * (origin - H.translate)) * inv;
		g_testRoot->local.scale = inv;
		// the pieces: from 3 units in front of his lips to the tip (a rounded end)
		const float rad = std::max(r.previewR, 0.2f), D = std::max(r.previewD, 0.5f);
		const float start = -3.0f, tipAt = std::max(D - rad, start);
		// pieces as long as they are thick, each as thick as the shape is there (0.4.1: a horse's flare and ring, a knot,
		// a bump show too)
		std::vector<std::pair<float, float>> at;   // where each piece starts, its thickness
		for (float y = start; y < tipAt - 0.05f && at.size() < 96;) {
			const float rr = rad * std::clamp(previewProfile(y + 0.5f * rad, D, r.previewShape), 0.3f, 3.0f);
			at.emplace_back(std::min(y, tipAt - rr), rr);
			y += std::max(rr * 0.8f, 0.1f);
		}
		const int need = std::max(static_cast<int>(at.size()), 1);
		while (static_cast<int>(g_testPieces.size()) < need) {
			auto* c = static_cast<NiAVObject*>(g_testSeg->Clone());
			if (!c)
				break;
			g_testPieces.emplace_back(c);
			g_testRoot->AttachChild(c, true);
		}
		for (int k = 0; k < static_cast<int>(g_testPieces.size()); ++k) {
			auto* p = g_testPieces[k].get();
			const bool use = k < need;
			p->SetAppCulled(!use);
			if (!use)
				continue;
			p->local = NiTransform();
			if (k < static_cast<int>(at.size())) {
				p->local.translate = NiPoint3(0.0f, at[k].first, 0.0f);
				p->local.scale = at[k].second;
			} else
				p->SetAppCulled(true);
		}
		if (g_testTipPiece) {
			g_testTipPiece->local = NiTransform();
			g_testTipPiece->local.translate = NiPoint3(0.0f, tipAt, 0.0f);
			g_testTipPiece->local.scale = rad * std::clamp(previewProfile(tipAt, D, r.previewShape), 0.3f, 3.0f);
		}
		NiUpdateData ud{};
		g_testRoot->Update(ud);
	}

	// 0.4.1: the test page tilts the player's head back (chin up) so his throat is easier to see.
	// 0.4.2: done in the game's own animation update, right after it poses him (DeepFit's frame task ran at another point
	// of the frame, so some frames were drawn with the tilt and some without - his head jittered up and down)
	std::atomic<float> g_tiltDeg{ 0.0f };

	void tiltNow(Actor* a, float deg)
	{
		auto* root = a ? a->Get3D(false) : nullptr;
		auto* h = root ? root->GetObjectByName("NPC Head [Head]") : nullptr;
		auto* par = h ? h->parent : nullptr;
		if (!par)
			return;
		const NiMatrix3 world = par->world.rotate * h->local.rotate;
		NiPoint3        u(world.entry[0][0], world.entry[1][0], world.entry[2][0]);   // his head's own right
		const float     lu = len(u);
		if (lu < 1e-4f)
			return;
		u = u * (1.0f / lu);
		const float a2 = deg * 0.01745329f, c = std::cos(a2), s = std::sin(a2), t = 1.0f - c;
		NiMatrix3   R;
		R.entry[0][0] = c + u.x * u.x * t;       R.entry[0][1] = u.x * u.y * t - u.z * s; R.entry[0][2] = u.x * u.z * t + u.y * s;
		R.entry[1][0] = u.y * u.x * t + u.z * s; R.entry[1][1] = c + u.y * u.y * t;       R.entry[1][2] = u.y * u.z * t - u.x * s;
		R.entry[2][0] = u.z * u.x * t - u.y * s; R.entry[2][1] = u.z * u.y * t + u.x * s; R.entry[2][2] = c + u.z * u.z * t;
		h->local.rotate = par->world.rotate.Transpose() * (R * world);
		NiUpdateData ud{};
		h->Update(ud);
	}

	// ------------------------------------------------------------------ 0.5.0: beards follow the mouth
	// A beard is its own mesh, hung on the head bone only: it did not move with the lips DeepFit moves or the jaw it opens.
	// Each beard vertex is matched (once) to the nearest few points of the face under it, and every frame it is moved as
	// far as those face points are moved by DeepFit - its morphs (read from the same MFEE extension .tri files MFEE uses)
	// and its lip / jaw bones. Only DeepFit's own movement: the beard's own expressions and lip sync play as before.
	// A beard the game animates is a BSDynamicTriShape (its vertices on the CPU); a static beard (BSTriShape, no
	// expressions of its own) cannot be moved this way and is left alone (the menu says so).

	struct TriMorphs
	{
		std::uint32_t                                            nv = 0;
		std::unordered_map<std::string, std::vector<NiPoint3>>   d;   // DeepFit's morphs only: name -> delta per vertex at 100
	};

	std::string lowerPath(std::string s)
	{
		for (auto& c : s) {
			c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
			if (c == '/')
				c = '\\';
		}
		while (!s.empty() && (s.front() == ' ' || s.front() == '\t'))
			s.erase(s.begin());
		while (!s.empty() && (s.back() == ' ' || s.back() == '\t' || s.back() == '\r'))
			s.pop_back();
		if (s.rfind("meshes\\", 0) == 0)
			s = s.substr(7);
		return s;
	}

	// 0.5.2: a file as the game sees it - loose, or packed in a BSA (vanilla beards' .tri files are in Skyrim - Meshes)
	std::vector<char> readGameFile(const std::string& relToMeshes)
	{
		std::vector<char>        out;
		BSResourceNiBinaryStream st(("meshes\\" + relToMeshes).c_str());
		static char buf[65536];
		for (; st.good();) {   // (the last piece is short: how much came is told by where the stream now is)
			const std::uint32_t at = st.tell();
			const bool          full = st.read(buf, static_cast<std::uint32_t>(sizeof(buf)));
			const std::uint32_t n = std::min<std::uint32_t>(st.tell() - at, static_cast<std::uint32_t>(sizeof(buf)));
			out.insert(out.end(), buf, buf + n);
			if (!full || n == 0 || out.size() > (64u << 20))
				break;
		}
		if (out.empty()) {   // a loose file the plain way, if the game's reader gave nothing
			std::ifstream in("Data\\meshes\\" + relToMeshes, std::ios::binary);
			if (in)
				out.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
		}
		return out;
	}

	// a .tri file (FRTRI003): its morphs that DeepFit drives
	std::shared_ptr<TriMorphs> loadTri(const std::string& rel)
	{
		static std::unordered_map<std::string, std::shared_ptr<TriMorphs>> cache;
		const std::string key = lowerPath(rel);
		if (auto it = cache.find(key); it != cache.end())
			return it->second;
		auto& out = cache[key];
		std::vector<char> b = readGameFile(key);   // (0.5.2: loose or in a BSA)
		auto i32 = [&](std::size_t o) { std::int32_t v = 0; if (o + 4 <= b.size()) std::memcpy(&v, b.data() + o, 4); return v; };
		if (b.size() < 64 || std::memcmp(b.data(), "FRTRI003", 8) != 0)
			return out;
		const std::int32_t nv = i32(8), nt = i32(12), nuv = i32(28), fl = i32(32), nm = i32(36), nmv = i32(44);
		if (nv <= 0 || nv > 200000 || nm < 0 || nm > 4000)
			return out;
		std::size_t o = 64 + static_cast<std::size_t>(nv) * 12 + static_cast<std::size_t>(std::max(nmv, 0)) * 12 + static_cast<std::size_t>(std::max(nt, 0)) * 12;
		if (fl & 1)
			o += static_cast<std::size_t>(std::max(nuv, 0)) * 8 + static_cast<std::size_t>(std::max(nt, 0)) * 12;
		static const std::unordered_set<std::string> ours = [] {
			std::unordered_set<std::string> s;
			for (auto* n : kMorphNames)
				s.insert(n);
			return s;
		}();
		auto t = std::make_shared<TriMorphs>();
		t->nv = static_cast<std::uint32_t>(nv);
		for (std::int32_t m = 0; m < nm; ++m) {
			const std::int32_t l = i32(o);
			if (l <= 0 || l > 512 || o + 4 + l + 4 + static_cast<std::size_t>(nv) * 6 > b.size())
				break;
			std::string name(b.data() + o + 4, b.data() + o + 4 + l);
			while (!name.empty() && name.back() == '\0')
				name.pop_back();
			o += 4 + l;
			float k = 0.0f;
			std::memcpy(&k, b.data() + o, 4);
			o += 4;
			if (ours.contains(name)) {
				auto& v = t->d[name];
				v.resize(nv);
				for (std::int32_t j = 0; j < nv; ++j) {
					std::int16_t s[3];
					std::memcpy(s, b.data() + o + static_cast<std::size_t>(j) * 6, 6);
					v[j] = NiPoint3(s[0] * k, s[1] * k, s[2] * k);
				}
			}
			o += static_cast<std::size_t>(nv) * 6;
		}
		out = t;
		return out;
	}

	// MFEE's ExtensionFile lines: a head's own .tri -> the .tri files that add morphs to it
	const std::unordered_map<std::string, std::vector<std::string>>& mfeeExtensions()
	{
		static std::unordered_map<std::string, std::vector<std::string>> map;
		static bool                                                     read = false;
		if (read)
			return map;
		read = true;
		std::error_code ec;
		for (auto& e : std::filesystem::directory_iterator("Data\\SKSE\\Plugins\\MuFacialExpressionExtended", ec)) {
			if (!e.is_regular_file(ec) || e.path().extension() != ".ini")
				continue;
			std::ifstream in(e.path());
			std::string   line;
			while (std::getline(in, line)) {
				const auto eq = line.find('=');
				if (eq == std::string::npos || lowerPath(line.substr(0, eq)) != "extensionfile")
					continue;
				const std::string rest = line.substr(eq + 1);
				const auto        comma = rest.find(',');
				if (comma == std::string::npos)
					continue;
				map[lowerPath(rest.substr(0, comma))].push_back(lowerPath(rest.substr(comma + 1)));
			}
		}
		return map;
	}

	// a mesh's point in his head bone's space, and its rotation / scale there (both meshes are skinned to that bone)
	bool headBind(BSGeometry* g, NiTransform& out)
	{
		auto* si = g ? g->GetGeometryRuntimeData().skinInstance.get() : nullptr;
		if (!si || !si->skinData || !si->bones || !si->skinData->boneData)
			return false;
		const std::uint32_t nb = si->skinData->bones;
		for (std::uint32_t i = 0; i < nb; ++i)
			if (si->bones[i] && si->bones[i]->name == "NPC Head [Head]") {
				out = si->skinData->boneData[i].skinToBone;
				return true;
			}
		return false;
	}

	// a dynamic mesh's vertices as they are now (its CPU copy)
	bool dynVerts(BSGeometry* g, std::vector<NiPoint3>& out)
	{
		auto* d = g ? g->AsDynamicTriShape() : nullptr;
		if (!d)
			return false;
		auto& rd = d->GetDynamicTrishapeRuntimeData();
		if (!rd.dynamicData || rd.dataSize < 16)
			return false;
		BSSpinLockGuard lk(rd.lock);
		const std::uint32_t n = rd.dataSize / 16;
		const float*        v = static_cast<const float*>(rd.dynamicData);
		out.resize(n);
		for (std::uint32_t i = 0; i < n; ++i)
			out[i] = NiPoint3(v[i * 4], v[i * 4 + 1], v[i * 4 + 2]);
		return true;
	}

	// the face mesh his lips are on, and its DeepFit morphs / bone weights - read once per face
	bool readHeadForBeards(Receiver& r, Actor* act, NiAVObject* root)
	{
		auto& hb = r.beardHead;
		hb = Receiver::BeardHead{};
		auto* base = act->GetActorBase();
		auto* hp = base ? base->GetCurrentHeadPartByType(BGSHeadPart::HeadPartType::kFace) : nullptr;
		if (!hp)
			return false;
		auto* o = root->GetObjectByName(hp->formEditorID);
		auto* g = o ? o->AsGeometry() : nullptr;
		if (!g || !headBind(g, hb.bind) || !dynVerts(g, hb.rest))
			return false;
		hb.geo = NiPointer<BSGeometry>(g);
		// 0.5.1: its expression .tri (MaleHead.tri - what MFEE's ExtensionFile lines name) is the head part's second one;
		// 0.5.0 looked only at the first (its race morphs, MaleHeadRaces.tri) and found none of DeepFit's morphs, so the
		// beard did not follow the jaw
		{
			auto& ext = mfeeExtensions();
			std::vector<std::string> files;
			for (auto mi : { BGSHeadPart::MorphIndex::kDefaultMorph, BGSHeadPart::MorphIndex::kRaceMorph, BGSHeadPart::MorphIndex::kChargenMorph }) {
				const char* tri = hp->morphs[mi].GetModel();
				if (!tri || !*tri)
					continue;
				if (hb.tri.empty())
					hb.tri = tri;
				files.push_back(lowerPath(tri));
				if (auto it = ext.find(lowerPath(tri)); it != ext.end())
					files.insert(files.end(), it->second.begin(), it->second.end());
			}
			for (auto& f : files) {
				auto t = loadTri(f);
				if (!t || t->nv != hb.rest.size())
					continue;
				for (int m = 0; m < kMorphs; ++m)
					if (auto mi = t->d.find(kMorphNames[m]); mi != t->d.end() && hb.morph[m].empty())
						hb.morph[m] = mi->second;
			}
		}
		// his DeepFit bones' weights on each face vertex (the skin's vertex data; bones numbered over the whole mesh or
		// within each partition - whichever puts the vertices on their own bones)
		auto* si = g->GetGeometryRuntimeData().skinInstance.get();
		auto* sd = si ? si->skinData.get() : nullptr;
		auto* sp = si ? si->skinPartition.get() : nullptr;
		if (!sd || !sp || sp->numPartitions == 0 || !si->bones || !sd->boneData)
			return true;
		const std::uint32_t nb = sd->bones;
		std::vector<int>    specOf(nb, -1);
		for (std::uint32_t b = 0; b < nb; ++b)
			for (int i = 0; i < r.nspecs(); ++i)
				if (r.bones[i].valid && si->bones[b] == r.bones[i].node.get())
					specOf[b] = i;
		auto& part = sp->partitions[0];
		if (!part.buffData || !part.buffData->rawVertexData || !(part.vertexDesc.GetFlags() & BSGraphics::Vertex::VF_SKINNED))
			return true;
		const std::uint32_t vSize = part.vertexDesc.GetSize();
		const std::uint32_t skinOff = part.vertexDesc.GetAttributeOffset(BSGraphics::Vertex::VA_SKINNING);
		const std::uint32_t nv = std::min<std::uint32_t>(sp->vertexCount, static_cast<std::uint32_t>(hb.rest.size()));
		std::vector<std::int16_t> partOf;
		if (sp->numPartitions > 1) {
			partOf.assign(sp->vertexCount, -1);
			for (std::uint32_t pi = 0; pi < sp->numPartitions; ++pi) {
				auto&                pp = sp->partitions[pi];
				const std::uint16_t* idx = (pp.buffData && pp.buffData->rawIndexData) ? pp.buffData->rawIndexData : pp.triList;
				if (!idx)
					continue;
				for (std::uint32_t t = 0; t < static_cast<std::uint32_t>(pp.triangles) * 3u; ++t)
					if (idx[t] < sp->vertexCount && partOf[idx[t]] < 0)
						partOf[idx[t]] = static_cast<std::int16_t>(pi);
			}
		}
		auto boneOf = [&](std::uint32_t j, std::uint8_t raw, bool local) -> std::uint32_t {
			if (local && !partOf.empty() && partOf[j] >= 0) {
				auto& pp = sp->partitions[partOf[j]];
				return (pp.bones && raw < pp.numBones) ? pp.bones[raw] : 0xFFFFu;
			}
			return raw;
		};
		// which numbering: the median distance from each vertex to its main bone's rest place (mesh space)
		auto offBones = [&](bool local) {
			std::vector<float> dd;
			for (std::uint32_t j = 0; j < nv; j += std::max<std::uint32_t>(1, nv / 200)) {
				const std::uint8_t*  v = part.buffData->rawVertexData + static_cast<std::size_t>(j) * vSize;
				const std::uint16_t* hw = reinterpret_cast<const std::uint16_t*>(v + skinOff);
				const std::uint8_t*  bi = reinterpret_cast<const std::uint8_t*>(hw + 4);
				int                  kb = 0;
				for (int k = 1; k < 4; ++k)
					if (half2float(hw[k]) > half2float(hw[kb])) kb = k;
				const std::uint32_t b = boneOf(j, bi[kb], local);
				if (b >= nb)
					continue;
				const NiTransform inv = sd->boneData[b].skinToBone.Invert();
				dd.push_back(len(hb.rest[j] - inv.translate));
			}
			if (dd.empty())
				return 1e9f;
			std::nth_element(dd.begin(), dd.begin() + dd.size() / 2, dd.end());
			return dd[dd.size() / 2];
		};
		const bool local = !partOf.empty() && offBones(true) < offBones(false) * 0.7f;
		hb.bw.assign(nv, {});
		int weighted = 0;
		for (std::uint32_t j = 0; j < nv; ++j) {
			const std::uint8_t*  v = part.buffData->rawVertexData + static_cast<std::size_t>(j) * vSize;
			const std::uint16_t* hw = reinterpret_cast<const std::uint16_t*>(v + skinOff);
			const std::uint8_t*  bi = reinterpret_cast<const std::uint8_t*>(hw + 4);
			float                tot = 0.0f;
			for (int k = 0; k < 4; ++k)
				tot += std::max(half2float(hw[k]), 0.0f);
			if (tot <= 0.0f)
				continue;
			auto& e = hb.bw[j];
			for (int k = 0; k < 4; ++k) {
				const float         w = std::max(half2float(hw[k]), 0.0f) / tot;
				const std::uint32_t b = boneOf(j, bi[k], local);
				if (w > 0.001f && b < nb && specOf[b] >= 0) {
					e.spec[e.n] = static_cast<std::int8_t>(specOf[b]);
					e.w[e.n] = w;
					++e.n;
				}
			}
			if (e.n)
				++weighted;
		}
		hb.weighted = weighted;
		if (Config::get().debugLog)
			logs::info("DeepFit: beards - {:08X}'s face {} ({} vertices): {} DeepFit morphs found in its .tri files ({}), {} vertices on DeepFit's bones",
				act->GetFormID(), g->name.c_str() ? g->name.c_str() : "?", hb.rest.size(),
				std::count_if(std::begin(hb.morph), std::end(hb.morph), [](auto& v) { return !v.empty(); }), hb.tri, weighted);
		return true;
	}

	// 0.5.2: the beard is moved by MFEE, together with his face. 0.5.0/0.5.1 wrote the moved beard into its vertices every
	// frame, but the game put its own back after that every frame (the log: "rewrote it" on every other update), so it never
	// showed. Now each beard gets an MFEE extension .tri of its own - DeepFit's morphs as the face under each beard point
	// moves with them - and an ExtensionFile line in DeepFit_Beards.ini. MFEE reads its ini when the game starts: a beard
	// DeepFit has not seen before follows the mouth from the next game start on.
	constexpr const char* kBeardIni = "Data\\SKSE\\Plugins\\MuFacialExpressionExtended\\DeepFit_Beards.ini";


	// the beards MFEE was told about when the game started (read once, before DeepFit adds any)
	const std::unordered_set<std::string>& beardsAtStart()
	{
		static std::unordered_set<std::string> s;
		static bool                            read = false;
		if (!read) {
			read = true;
			std::ifstream in(kBeardIni);
			std::string   line;
			while (std::getline(in, line)) {
				const auto eq = line.find('='), comma = line.find(',');
				if (eq != std::string::npos && comma != std::string::npos && comma > eq && lowerPath(line.substr(0, eq)) == "extensionfile")
					s.insert(lowerPath(line.substr(eq + 1, comma - eq - 1)));
			}
		}
		return s;
	}

	// DeepFit_Beards.ini: the line for this beard's .tri put in (or taken out)
	void beardIniLine(const std::string& baseTri, const std::string& ourTri, bool want)
	{
		std::vector<std::string> lines;
		{
			std::ifstream in(kBeardIni);
			std::string   line;
			while (std::getline(in, line)) {
				if (!line.empty() && line.back() == '\r')
					line.pop_back();
				lines.push_back(line);
			}
		}
		const std::string key = lowerPath(baseTri);
		bool              has = false, changed = false;
		for (auto it = lines.begin(); it != lines.end();) {
			const auto eq = it->find('='), comma = it->find(',');
			const bool ours = eq != std::string::npos && comma != std::string::npos && comma > eq && lowerPath(it->substr(eq + 1, comma - eq - 1)) == key;
			if (ours && !want) {
				it = lines.erase(it);
				changed = true;
				continue;
			}
			has |= ours;
			++it;
		}
		if (want && !has) {
			if (lines.empty())
				lines.push_back("; DeepFit 0.5.2: beards follow the mouth - written by DeepFit (one line per beard it has seen)");
			lines.push_back("ExtensionFile = " + baseTri + ", " + ourTri);
			changed = true;
		}
		if (!changed)
			return;
		std::error_code ec;
		std::filesystem::create_directories(std::filesystem::path(kBeardIni).parent_path(), ec);
		std::ofstream out(kBeardIni, std::ios::binary | std::ios::trunc);
		for (auto& l : lines)
			out << l << "\r\n";
	}

	// the beard's own extension .tri: its expression .tri's vertices / triangles / UVs and DeepFit's morphs
	void makeBeardTri(Receiver& r, BGSHeadPart* hp, Receiver::Beard& b)
	{
		const auto& cfg = Config::get();
		const char* tri = hp->morphs[BGSHeadPart::MorphIndex::kDefaultMorph].GetModel();
		if (!tri || !*tri) {
			b.state = "no expression .tri - it cannot move";
			return;
		}
		std::string name = lowerPath(tri);
		for (auto& c : name)
			if (c == '\\' || c == ':' || c == ' ')
				c = '_';
		const std::string ourRel = "DeepFit\\Beards\\" + name;
		if (!cfg.beardFollow) {
			beardIniLine(tri, ourRel, false);
			b.state = "following is off";
			return;
		}
		std::vector<char> src = readGameFile(lowerPath(tri));
		auto i32 = [&](std::size_t o) { std::int32_t v = 0; if (o + 4 <= src.size()) std::memcpy(&v, src.data() + o, 4); return v; };
		if (src.size() < 64 || std::memcmp(src.data(), "FRTRI003", 8) != 0) {
			b.state = "its .tri could not be found - it cannot move";
			return;
		}
		const std::int32_t nv = i32(8), nt = i32(12), nuv = i32(28), fl = i32(32), nmv = i32(44);
		if (nv <= 0 || static_cast<std::uint32_t>(nv) != b.n) {
			b.state = "its .tri does not match the mesh - it cannot move";
			return;
		}
		const std::size_t vBytes = static_cast<std::size_t>(nv) * 12, mvBytes = static_cast<std::size_t>(std::max(nmv, 0)) * 12;
		const std::size_t restBytes = static_cast<std::size_t>(std::max(nt, 0)) * 12 + ((fl & 1) ? static_cast<std::size_t>(std::max(nuv, 0)) * 8 + static_cast<std::size_t>(std::max(nt, 0)) * 12 : 0);
		if (64 + vBytes + mvBytes + restBytes > src.size()) {
			b.state = "its .tri could not be read - it cannot move";
			return;
		}
		const auto&       hb = r.beardHead;
		// 0.5.6: his face read before the game had hung its skin on his bones ("0 vertices on DeepFit's bones" in the log,
		// every other read): a .tri made then has none of the lip bones' morphs, and the file went back and forth between
		// the two each time - whichever was there at the game start was the one used (a beard that stayed shut over his
		// mouth). Not written until his face's bones are there
		if (cfg.beardBones && hb.weighted == 0 && r.found > 0) {
			b.state = "waiting for his face's bones";
			return;
		}
		const float       reach = std::max(cfg.beardReach, 0.1f);
		std::vector<char> out(src.begin(), src.begin() + 64);
		out.insert(out.end(), src.begin() + 64, src.begin() + 64 + vBytes);                                  // vertices
		out.insert(out.end(), src.begin() + 64 + vBytes + mvBytes, src.begin() + 64 + vBytes + mvBytes + restBytes);   // triangles, UVs
		std::int32_t nm = 0;
		std::vector<NiPoint3> d(nv);
		for (int m = 0; m < kMorphs; ++m) {
			if (hb.morph[m].empty())
				continue;
			float mx = 0.0f;
			for (std::int32_t i = 0; i < nv; ++i) {
				const float fade = std::clamp(2.0f - b.dist[i] / reach, 0.0f, 1.0f);
				NiPoint3    h;
				if (fade > 0.0f)
					for (int k = 0; k < 3; ++k) {
						const std::uint32_t j = b.hv[i * 3 + k];
						if (j < hb.morph[m].size())
							h += hb.morph[m][j] * b.hw[i * 3 + k];
					}
				d[i] = (b.toBeard * h) * (b.toBeardK * cfg.beardStrength * fade);
				mx = std::max({ mx, std::fabs(d[i].x), std::fabs(d[i].y), std::fabs(d[i].z) });
			}
			if (mx < 1e-4f)
				continue;   // nothing of the beard is over this part of the face
			const float          k = mx / 32767.0f;
			const std::string    mn = kMorphNames[m];
			const std::int32_t   len = static_cast<std::int32_t>(mn.size() + 1);
			auto put = [&](const void* p, std::size_t n) { out.insert(out.end(), static_cast<const char*>(p), static_cast<const char*>(p) + n); };
			put(&len, 4);
			put(mn.c_str(), mn.size() + 1);
			put(&k, 4);
			for (std::int32_t i = 0; i < nv; ++i) {
				const std::int16_t s[3] = { static_cast<std::int16_t>(std::lround(d[i].x / k)), static_cast<std::int16_t>(std::lround(d[i].y / k)),
					static_cast<std::int16_t>(std::lround(d[i].z / k)) };
				put(s, 6);
			}
			++nm;
		}
		// 0.5.4: the lip bones' morphs (see kBeardBoneUnits)
		if (cfg.beardBones && !hb.bw.empty() && r.head) {
			const NiTransform& H = r.head->world;
			const float        hs = H.scale > 1e-6f ? H.scale : 1.0f;
			// (0.5.5: his face's axes worked out here from his head, as the fitting does - r.fx/fy/fz are only set once the
			// mouth has been fitted, and before that pointed the world's way: each game start wrote a different .tri, so the
			// beard always waited for "the next game start" and moved the wrong way meanwhile)
			NiPoint3 FX = norm(H.rotate * NiPoint3(1, 0, 0));
			if (r.bones[6].valid && r.bones[7].valid) {
				const NiPoint3 across = (r.bones[7].parent->world * r.bones[7].restLocal) - (r.bones[6].parent->world * r.bones[6].restLocal);
				if (len(across) > 0.05f)
					FX = norm(across);
			}
			NiPoint3 FF = r.hmag ? r.hmag->world.translate - H.translate : H.rotate * NiPoint3(0, 1, 0);
			FF = FF - FX * dot(FF, FX);
			const NiPoint3 FY = len(FF) > 0.01f ? norm(FF) : norm(H.rotate * NiPoint3(0, 1, 0));
			NiPoint3       FZ = norm(NiPoint3(FX.y * FY.z - FX.z * FY.y, FX.z * FY.x - FX.x * FY.z, FX.x * FY.y - FX.y * FY.x));
			if (r.bones[kUpperCenter].valid && r.bones[kBottomCenter].valid &&
				dot((r.bones[kUpperCenter].parent->world * r.bones[kUpperCenter].restLocal) - (r.bones[kBottomCenter].parent->world * r.bones[kBottomCenter].restLocal), FZ) < 0.0f) {
				FX = FX * -1.0f;
				FZ = FZ * -1.0f;
			}
			for (int sp = 0; sp < r.nspecs() && sp < kMaxSpecs; ++sp) {
				const auto kd = r.specs[sp].kind;
				if (kd != Kind::Lip && kd != Kind::Corner && kd != Kind::Jaw)
					continue;
				const NiPoint3 dn = norm(r.specs[sp].dir);
				const NiPoint3 world = FX * dn.x + FY * dn.y + FZ * dn.z;
				NiPoint3       headDir = (H.rotate.Transpose() * world) * (1.0f / hs);   // 1 unit in the world, in head-bone space
				// (rounded: the face axes wobble a hair from frame to frame - the .tri would differ every time it is made)
				headDir = NiPoint3(std::round(headDir.x * 50.0f) / 50.0f, std::round(headDir.y * 50.0f) / 50.0f, std::round(headDir.z * 50.0f) / 50.0f);
				// head bone -> beard mesh: the beard's own bind (as toBeard, which also carries the face mesh's into head space)
				const NiPoint3 beardDir = b.toBeardRot * headDir * b.toBeardHeadK;
				float          mx = 0.0f;
				for (std::int32_t i = 0; i < nv; ++i) {
					const float fade = std::clamp(2.0f - b.dist[i] / reach, 0.0f, 1.0f);
					float       w = 0.0f;
					if (fade > 0.0f)
						for (int k = 0; k < 3; ++k) {
							const std::uint32_t j = b.hv[i * 3 + k];
							if (j < hb.bw.size())
								for (int q = 0; q < hb.bw[j].n; ++q)
									if (hb.bw[j].spec[q] == sp)
										w += hb.bw[j].w[q] * b.hw[i * 3 + k];
						}
					d[i] = beardDir * (w * fade * cfg.beardStrength * kBeardBoneUnits);
					mx = std::max({ mx, std::fabs(d[i].x), std::fabs(d[i].y), std::fabs(d[i].z) });
				}
				if (mx < 1e-3f)
					continue;
				const float k = mx / 32767.0f;
				auto put = [&](const void* p, std::size_t n) { out.insert(out.end(), static_cast<const char*>(p), static_cast<const char*>(p) + n); };
				for (int in = 0; in < 2; ++in) {
					const std::string& mn = beardBoneName(sp, in == 1);
					const std::int32_t len = static_cast<std::int32_t>(mn.size() + 1);
					put(&len, 4);
					put(mn.c_str(), mn.size() + 1);
					put(&k, 4);
					const float sg = in ? -1.0f : 1.0f;
					for (std::int32_t i = 0; i < nv; ++i) {
						const std::int16_t sv[3] = { static_cast<std::int16_t>(std::lround(sg * d[i].x / k)), static_cast<std::int16_t>(std::lround(sg * d[i].y / k)),
							static_cast<std::int16_t>(std::lround(sg * d[i].z / k)) };
						put(sv, 6);
					}
					++nm;
				}
				r.beardBones |= (1ull << sp);
			}
		}
		const std::int32_t zero = 0;
		std::memcpy(out.data() + 36, &nm, 4);     // morphs: DeepFit's
		std::memcpy(out.data() + 40, &zero, 4);   // no modifiers ...
		std::memcpy(out.data() + 44, &zero, 4);   // ... nor their vertices
		const std::filesystem::path path = std::filesystem::path("Data\\meshes") / ourRel;
		bool same = false;
		{
			std::ifstream old(path, std::ios::binary);
			if (old) {
				std::vector<char> o((std::istreambuf_iterator<char>(old)), std::istreambuf_iterator<char>());
				same = o == out;
			}
		}
		if (!same) {
			std::error_code ec;
			std::filesystem::create_directories(path.parent_path(), ec);
			std::ofstream f(path, std::ios::binary | std::ios::trunc);
			f.write(out.data(), static_cast<std::streamsize>(out.size()));
		}
		const bool live = beardsAtStart().contains(lowerPath(tri));
		beardIniLine(tri, ourRel, true);
		b.state = !nm ? "nothing of it is over the moving part of the face" :
		          live && same ? "follows the mouth (through MFEE)" :
		          live         ? "follows the mouth - its new settings after the next game start" :
		                         "ready - follows the mouth from the next game start";
	}

	// his beard meshes (facial hair head parts, and their extra parts), matched to his face
	void findBeards(Receiver& r, Actor* act, NiAVObject* root)
	{
		r.beards.clear();
		auto* base = act->GetActorBase();
		if (!base || !r.beardHead.geo)
			return;
		std::vector<BGSHeadPart*> parts;
		std::function<void(BGSHeadPart*)> add = [&](BGSHeadPart* hp) {
			if (!hp || std::find(parts.begin(), parts.end(), hp) != parts.end())
				return;
			parts.push_back(hp);
			for (auto* x : hp->extraParts)
				add(x);
		};
		if (base->headParts)
			for (std::uint8_t i = 0; i < base->numHeadParts; ++i)
				if (auto* hp = base->headParts[i]; hp && hp->type.get() == BGSHeadPart::HeadPartType::kFacialHair)
					add(hp);
		if (auto* hp = base->GetCurrentHeadPartByType(BGSHeadPart::HeadPartType::kFacialHair))
			add(hp);
		const auto&       hb = r.beardHead;
		const NiTransform headInv = hb.bind;   // face mesh -> head bone
		std::vector<NiPoint3> headPts(hb.rest.size());
		for (std::size_t j = 0; j < hb.rest.size(); ++j)
			headPts[j] = headInv * hb.rest[j];
		// 0.5.8: which face points belong to his lower jaw (they move with the jaw-open morph): the main direction the
		// morph moves things, and the points that move at least a quarter of the most along it
		std::vector<bool> lowerFace;
		if (!hb.morph[0].empty() && hb.morph[0].size() == hb.rest.size()) {
			NiPoint3 dir;
			float    mx = 0.0f;
			for (auto& v : hb.morph[0])
				mx = std::max(mx, len(v));
			for (auto& v : hb.morph[0])
				if (len(v) > 0.5f * mx)
					dir += v;
			if (mx > 1e-4f && len(dir) > 1e-6f) {
				dir = norm(dir);
				float top = 0.0f;
				for (auto& v : hb.morph[0])
					top = std::max(top, dot(v, dir));
				lowerFace.resize(hb.rest.size());
				for (std::size_t j = 0; j < hb.rest.size(); ++j)
					lowerFace[j] = dot(hb.morph[0][j], dir) > 0.25f * top;
			}
		}
		for (auto* hp : parts) {
			auto* o = root->GetObjectByName(hp->formEditorID);
			auto* g = o ? o->AsGeometry() : nullptr;
			Receiver::Beard b;
			b.name = hp->formEditorID.c_str() ? hp->formEditorID.c_str() : "?";
			if (!g) {
				b.state = "not on him";
				r.beards.push_back(std::move(b));
				continue;
			}
			b.geo = NiPointer<BSGeometry>(g);
			std::vector<NiPoint3> pts;
			NiTransform           bind;
			if (!g->AsDynamicTriShape() || !dynVerts(g, pts)) {
				b.state = "static - it cannot move";
				r.beards.push_back(std::move(b));
				continue;
			}
			if (!headBind(g, bind)) {
				b.state = "not on his head bone";
				r.beards.push_back(std::move(b));
				continue;
			}
			// face-mesh moves into the beard's mesh space: beard <- head bone <- face
			const NiTransform beardInv = bind.Invert();
			b.toBeard = beardInv.rotate * hb.bind.rotate;
			b.toBeardK = beardInv.scale * hb.bind.scale;
			b.toBeardRot = beardInv.rotate;
			b.toBeardHeadK = beardInv.scale;
			b.n = static_cast<std::uint32_t>(pts.size());
			b.hv.assign(b.n * 3, 0);
			b.hw.assign(b.n * 3, 0.0f);
			b.dist.assign(b.n, 1e9f);
			for (std::uint32_t i = 0; i < b.n; ++i) {
				const NiPoint3 p = bind * pts[i];
				// 0.5.8: the 6 nearest face points, then only those on one side of his mouth (the side most of them are
				// on): a beard point at his mouth line took the upper and the lower lip half each and stayed in the
				// middle when the mouth opened - a curtain of beard across the open mouth
				constexpr int  K = 6;
				std::uint32_t  cand[K] = {};
				float          cd[K];
				std::fill(std::begin(cd), std::end(cd), 1e30f);
				for (std::uint32_t j = 0; j < headPts.size(); ++j) {
					const NiPoint3 d = headPts[j] - p;
					const float    q = d.x * d.x + d.y * d.y + d.z * d.z;
					if (q >= cd[K - 1])
						continue;
					int k = K - 1;
					while (k > 0 && q < cd[k - 1]) {
						cd[k] = cd[k - 1];
						cand[k] = cand[k - 1];
						--k;
					}
					cd[k] = q;
					cand[k] = j;
				}
				bool lowerSide = false;
				if (!lowerFace.empty()) {
					float vote = 0.0f;
					for (int k = 0; k < K; ++k)
						if (cd[k] < 1e29f)
							vote += (lowerFace[cand[k]] ? 1.0f : -1.0f) / (std::sqrt(cd[k]) + 0.05f);
					lowerSide = vote > 0.0f;
				}
				std::uint32_t best[3] = { cand[0], cand[0], cand[0] };
				float         bd[3] = { cd[0], cd[0], cd[0] };
				{
					int got = 0;
					for (int k = 0; k < K && got < 3; ++k)
						if (cd[k] < 1e29f && (lowerFace.empty() || lowerFace[cand[k]] == lowerSide)) {
							best[got] = cand[k];
							bd[got] = cd[k];
							++got;
						}
					for (int k = got; k < 3 && got > 0; ++k) {   // fewer than three on that side: repeat the nearest one
						best[k] = best[0];
						bd[k] = bd[0];
					}
				}
				float tot = 0.0f, w[3];
				for (int k = 0; k < 3; ++k) {
					w[k] = 1.0f / (std::sqrt(bd[k]) + 0.05f);
					tot += w[k];
				}
				for (int k = 0; k < 3; ++k) {
					b.hv[i * 3 + k] = best[k];
					b.hw[i * 3 + k] = w[k] / tot;
				}
				b.dist[i] = std::sqrt(bd[0]);
			}
			makeBeardTri(r, hp, b);   // 0.5.2
			r.beards.push_back(std::move(b));
		}
		if (Config::get().debugLog)
			for (auto& b : r.beards)
				logs::info("DeepFit: beards - {:08X}'s {} ({} vertices): {}", act->GetFormID(), b.name, b.n, b.state);
	}

	void beardsFor(Receiver& r, Actor* act, float delta)
	{
		const auto& cfg = Config::get();
		auto*       root = act->Get3D(false);
		// shaved for the scene (the player only)
		const bool shave = cfg.beardShave && act->IsPlayerRef() && r.inScene;
		r.beardScan -= delta;
		if (!root)
			return;
		if (r.beardScan <= 0.0f || (r.beardHead.geo && !r.beardHead.geo->parent)) {   // every few seconds, and when his face is rebuilt
			r.beardScan = 4.0f;
			auto* base = act->GetActorBase();
			auto* fhp = base ? base->GetCurrentHeadPartByType(BGSHeadPart::HeadPartType::kFace) : nullptr;
			auto* fo = fhp ? root->GetObjectByName(fhp->formEditorID) : nullptr;
			// (0.5.1: also when it was read before his mouth bones were found - none of its vertices on them)
			int morphs = 0;   // (0.5.2: read again while none of DeepFit's morphs were found in its .tri files)
			for (auto& m : r.beardHead.morph)
				morphs += m.empty() ? 0 : 1;
			if (!r.beardHead.geo || fo != r.beardHead.geo.get() || morphs == 0) {
				for (auto& b : r.beards)
					if (b.geo && b.culled)
						b.geo->SetAppCulled(false);
				if (readHeadForBeards(r, act, root))
					findBeards(r, act, root);
				else
					r.beards.clear();
			} else {   // a beard put on or taken off?
				std::size_t now = 0;
				if (base && base->headParts)
					for (std::uint8_t i = 0; i < base->numHeadParts; ++i)
						if (auto* hp = base->headParts[i]; hp && hp->type.get() == BGSHeadPart::HeadPartType::kFacialHair)
							++now;
				bool gone = false;
				for (auto& b : r.beards)
					if (b.geo && !b.geo->parent)
						gone = true;
				if (gone || (now > 0) != !r.beards.empty())
					findBeards(r, act, root);
			}
		}
		for (auto& b : r.beards) {
			if (!b.geo)
				continue;
			if (shave != b.culled) {
				b.geo->SetAppCulled(shave);
				b.culled = shave;
			}
		}
	}

	// his beards given back as they were (his head let go of)
	void dropBeards(Receiver& r)
	{
		for (auto& b : r.beards)
			if (b.geo && b.culled) {
				b.geo->SetAppCulled(false);
				b.culled = false;
			}
	}

	struct AnimHook
	{
		static void Player(PlayerCharacter* a, float dt)
		{
			g_playerAnim(a, dt);
			const float deg = g_tiltDeg.load(std::memory_order_relaxed);
			if (a && deg > 0.01f)
				tiltNow(a, deg);
			applyFace(a);   // 0.4.2
		}
		static void Char(Character* a, float dt)
		{
			g_charAnim(a, dt);
			applyFace(a);   // 0.4.2: men other than the player
		}
		static inline REL::Relocation<decltype(Player)> g_playerAnim;
		static inline REL::Relocation<decltype(Char)>   g_charAnim;
		static void Install()
		{
			REL::Relocation<std::uintptr_t> vp{ PlayerCharacter::VTABLE[0] };
			REL::Relocation<std::uintptr_t> vc{ Character::VTABLE[0] };
			g_playerAnim = vp.write_vfunc(0x7D, Player);
			g_charAnim = vc.write_vfunc(0x7D, Char);
			logs::info("DeepFit: hooked the player's animation update (the test page's head tilt)");
		}
	};

	// 0.4.3: PPA bends the partner's schlong into his mouth after DeepFit has read it (it reads the animation's pose), so
	// when the animation leaves it a few units off - under his chin - the mouth never opened although on screen it was
	// in. When the shaft's line passes close by his lips, it is turned about its base onto them, the way PPA aims it.
	void aimAtMouth(std::vector<NiPoint3>& pts, const Partner& p, const NiPoint3& C, const Config& cfg)
	{
		if (!cfg.followAim || p.chain.size() < 2 || !p.chain.front() || !p.chain.back())
			return;
		// 0.4.5: why it was not turned (debug log, once a second) - a canine read 25 units off his lips was never aimed
		static double toldAt = -1e9;
		auto why = [&](const char* r, float a, float b) {
			if (cfg.debugLog && g_clock - toldAt > 1.0) {
				toldAt = g_clock;
				logs::info("DeepFit: not aimed at his lips - {} ({:.1f} / {:.1f})", r, a, b);
			}
		};
		const NiPoint3 B = wP(p, p.chain.front().get()).translate, T = wP(p, p.chain.back().get()).translate;
		const NiPoint3 d = T - B, v = C - B;
		const float    L = len(d), lv = len(v);
		if (L < 1.0f || lv < 1.0f || lv > L * 1.8f + 4.0f)
			return why("his lips are too far from the base of his schlong (distance / its length)", lv, L);
		const float t = dot(v, d) / (L * L);
		if (t < 0.15f)
			return why("his lips are behind or beside the base of his schlong (share along it)", t, 0.15f);
		const NiPoint3 perpV = v - d * t;
		if (len(perpV) > cfg.aimRadius)
			return why("its line passes too far from his lips (units / \"Aim when it passes within\")", len(perpV), cfg.aimRadius);
		// 0.5.6: how much of the turn it gets fades in and out near each limit - it was all or nothing, and deep in (his
		// lips near the base: the share along it about 0.3) it switched on and off from frame to frame: the schlong jumped
		// between the animation's aim and his lips, and his lips opened and snapped shut with it
		auto ramp = [](float x) { x = std::clamp(x, 0.0f, 1.0f); return x * x * (3.0f - 2.0f * x); };
		float share = ramp((t - 0.15f) / 0.2f) * ramp((cfg.aimRadius - len(perpV)) / std::max(cfg.aimRadius * 0.25f, 0.1f));
		const NiPoint3 a = norm(d), b = norm(v);
		NiPoint3       ax(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
		const float    s = len(ax), c = dot(a, b);
		const float    ang = std::atan2(s, c);
		if (ang > cfg.aimMaxDeg * 0.01745329f)
			return why("it would have to turn too far (degrees / \"Turn it at most\")", ang * 57.29578f, cfg.aimMaxDeg);
		{
			const float maxR = cfg.aimMaxDeg * 0.01745329f;
			share *= ramp((maxR - ang) / std::max(maxR * 0.25f, 0.01f));
		}
		if (s < 1e-5f || ang * share < 0.003f)
			return;
		ax = ax * (1.0f / s);
		const float cs = std::cos(ang * share), sn = std::sin(ang * share);
		for (auto& q : pts) {   // Rodrigues, about B
			const NiPoint3 r = q - B;
			const NiPoint3 k(ax.y * r.z - ax.z * r.y, ax.z * r.x - ax.x * r.z, ax.x * r.y - ax.y * r.x);
			q = B + r * cs + k * sn + ax * (dot(ax, r) * (1.0f - cs));
		}
	}

	// 0.4.4: his schlong's shape, found by DeepFit itself when Anatomica is not installed (Anatomica's own detection - its
	// compatibility files, menu choices, More Nasty Critters - is fuller, and when it is there it tells DeepFit instead).
	// In order: a keyword on his schlong (DeepFit_Shape_<kind> or Anatomica_Shape_<kind>), the schlong's name (a worn
	// one, or TNG's add-on he has), then his race. Kinds: 3 flared, 4 knotted, 5 tentacle, 6 ridged, 7 segmented, 0 plain.
	bool g_anatomica = false;   // AnatomicaMenu.dll is installed

	bool hasW(const std::string& n, std::initializer_list<const char*> ws)
	{
		for (auto* w : ws)
			if (n.find(w) != std::string::npos)
				return true;
		return false;
	}

	int kindFromWord(const std::string& raw)
	{
		std::string n = " " + raw + " ";
		for (auto& c : n)
			c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
		if (hasW(n, { "canine", "knot", "dog", "wolf", "fox", "husky" })) return 4;
		if (hasW(n, { "horse", "equine", "flare", "centaur", "ungulate" })) return 3;
		if (hasW(n, { "tentacle" })) return 5;
		if (hasW(n, { "segment", "insect", "chitin", "plated" })) return 7;
		if (hasW(n, { "argonian", "croc", "drake", "wyvern", "serpent", "reptil", "dragon", "ridge", "lizard", "shark", "khajiit", "feline", "barb", "sabre", "cat " })) return 6;
		if (hasW(n, { "uncut", "circumcis", "human", "regular", "muscular" })) return 0;
		return -1;
	}

	int kindFromKeywords(TESObjectARMO* armo)
	{
		if (!armo)
			return -1;
		for (std::uint32_t i = 0; i < armo->numKeywords; ++i) {
			auto* kw = armo->keywords[i];
			std::string e = kw && kw->formEditorID.c_str() ? kw->formEditorID.c_str() : "";
			for (auto& c : e)
				c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
			for (const char* pre : { "deepfit_shape_", "anatomica_shape_" })
				if (e.rfind(pre, 0) == 0) {
					const std::string k = e.substr(std::strlen(pre));
					if (k == "plain" || k == "uncut" || k == "cut") return 0;
					if (k == "flared" || k == "horse") return 3;
					if (k == "knotted" || k == "knot") return 4;
					if (k == "tentacle") return 5;
					if (k == "ridged" || k == "barbed") return 6;
					if (k == "segmented") return 7;
				}
		}
		return -1;
	}

	std::unordered_map<TESObjectARMA*, TESObjectARMO*> g_schlongByArma;   // every schlong armour (slot 52) by its armatures
	bool                                               g_schlongIndexed = false;

	int findKind(Actor* a, std::string& why)
	{
		why.clear();
		if (!a)
			return -1;
		if (!g_schlongIndexed) {
			g_schlongIndexed = true;
			if (auto* dh = TESDataHandler::GetSingleton())
				for (auto* armo : dh->GetFormArray<TESObjectARMO>()) {
					if (!armo || !armo->HasPartOf(BGSBipedObjectForm::BipedObjectSlot::kModPelvisSecondary) || armo->HasPartOf(BGSBipedObjectForm::BipedObjectSlot::kBody))
						continue;
					for (auto* arma : armo->armorAddons)
						if (arma && !g_schlongByArma.contains(arma))
							g_schlongByArma[arma] = armo;
				}
			logs::info("DeepFit: {} schlong armatures indexed (his shape)", g_schlongByArma.size());
		}
		auto byArmour = [&](TESObjectARMO* armo) -> int {
			if (!armo)
				return -1;
			const std::string nm = armo->GetName() ? armo->GetName() : "";
			if (int k = kindFromKeywords(armo); k >= 0) { why = "its keyword (" + nm + ")"; return k; }
			if (int k = kindFromWord(nm); k >= 0) { why = "its name (" + nm + ")"; return k; }
			return -1;
		};
		if (auto* worn = a->GetWornArmor(BGSBipedObjectForm::BipedObjectSlot::kModPelvisSecondary); worn && !worn->HasPartOf(BGSBipedObjectForm::BipedObjectSlot::kBody))
			if (int k = byArmour(worn); k >= 0)
				return k;
		if (auto* skin = a->GetSkin())
			for (auto* arma : skin->armorAddons)
				if (auto it = g_schlongByArma.find(arma); it != g_schlongByArma.end())
					if (int k = byArmour(it->second); k >= 0)
						return k;
		if (auto* race = a->GetRace()) {
			const std::string ed = race->GetFormEditorID() ? race->GetFormEditorID() : "";
			const std::string nm = race->GetName() ? race->GetName() : "";
			int k = kindFromWord(ed);
			if (k < 0) k = kindFromWord(nm);
			if (k < 0 && (ed.find("Werewolf") != std::string::npos || ed.find("werewolf") != std::string::npos)) k = 4;
			if (k > 0) { why = "his race (" + (nm.empty() ? ed : nm) + ")"; return k; }
		}
		why = "nothing says - plain";
		return 0;
	}

	const char* kindName(int k)
	{
		switch (k) {
		case 3: return "flared";
		case 4: return "knotted";
		case 5: return "tentacle";
		case 6: return "ridged";
		case 7: return "segmented";
		default: return "plain";
		}
	}

	// 0.4.0: what DeepFit found, for the menu
	struct StatusData
	{
		std::string text, race;
		double      at = -1e9;
	};
	StatusData g_status;
	std::mutex g_statusLock;

	// 0.1.2: driven from the input-event queue (sent every frame) through the SKSE task queue,
	// the way OStrands does it. PlayerCharacter::Update is not called while OStim has the player
	// in a scene (0.1.1 logged nothing from a scene's start to its end), so the mouth never moved.
	// 0.4.5: called at the start of every frame (input), before the game animates anyone: every bone a partner's shaft is
	// read from, as last frame ended (with PPA's aim)
	void snapPartners()
	{
		g_snap.clear();
		g_snapT.clear();
		g_useSnap = Config::get().readAfterPpa;
		if (!g_useSnap)
			return;
		for (auto& [id, p] : g_partners) {
			for (auto& c : p.chain)
				if (c)
					g_snap[c.get()] = c->world;
			for (auto& sh : p.shapes) {
				auto* geom = sh.geom.get();
				if (!geom)
					continue;
				auto* si = geom->GetGeometryRuntimeData().skinInstance.get();
				if (!si || !si->skinData || !si->bones)
					continue;
				const std::uint32_t nb = si->skinData->bones;
				for (std::uint32_t i = 0; i < nb; ++i)
					if (auto* b = si->bones[i])
						g_snap[b] = b->world;
				if (si->boneWorldTransforms)
					for (std::uint32_t i = 0; i < nb; ++i)
						if (const NiTransform* t = si->boneWorldTransforms[i])
							g_snapT[t] = *t;
			}
		}
	}

	class FrameDriver : public BSTEventSink<InputEvent*>
	{
	public:
		static FrameDriver* Get()
		{
			static FrameDriver d;
			return &d;
		}
		BSEventNotifyControl ProcessEvent(InputEvent* const* a_events, BSTEventSource<InputEvent*>*) override
		{
			// 0.3.0: the Mega mouth key (0.3.1: a combination - Shift / Ctrl / Alt held with it - and set in the menu by pressing it)
			if (a_events) {
				for (auto* e = *a_events; e; e = e->next) {
					auto* b = e->AsButtonEvent();
					if (!b || b->GetDevice() != INPUT_DEVICE::kKeyboard)
						continue;
					const int code = static_cast<int>(b->GetIDCode());
					const int mod = (code == 42 || code == 54) ? 1 : (code == 29 || code == 157) ? 2 : (code == 56 || code == 184) ? 4 : 0;
					if (mod) {
						if (b->IsPressed()) g_mods |= mod; else g_mods &= ~mod;
						continue;
					}
					if (!b->IsDown())
						continue;
					auto& cfg = Config::get();
					if (g_capture) {
						g_capture = false;
						if (code != 1) {   // Esc: leave it as it was
							cfg.megaKey = code;
							cfg.megaKeyMods = g_mods;
							cfg.save();
							logs::info("DeepFit: Mega mouth key set to {}", DeepFit::KeyText(code, g_mods));
						}
						continue;
					}
					if (cfg.megaKey <= 0 || code != cfg.megaKey || g_mods != cfg.megaKeyMods)
						continue;
					auto* ui = UI::GetSingleton();
					// (no typing check: CommonLibSSE-NG 3.7 has no ControlMap::GetRuntimeData; the console and paused menus cover it)
					if (ui && (ui->IsMenuOpen(Console::MENU_NAME) || ui->GameIsPaused()))
						continue;
					const bool on = !DeepFit::MegaMouthNow();
					DeepFit::SetMegaOverride(on ? 1 : 0);
					RE::DebugNotification(on ? "DeepFit: mega mouth on" : "DeepFit: mega mouth off");
					logs::info("DeepFit: mega mouth {} (key)", on ? "on" : "off");
				}
			}
			if (_queued.exchange(true))
				return BSEventNotifyControl::kContinue;
			snapPartners();   // 0.4.5: before the animation update (the task below runs after it)
			if (auto* t = SKSE::GetTaskInterface()) {
				t->AddTask([this]() {
					_queued = false;
					auto* ui = UI::GetSingleton();
					if (ui && (ui->GameIsPaused() || ui->IsMenuOpen(MainMenu::MENU_NAME)))
						return;
					DeepFit::OnFrame(GetSecondsSinceLastFrame());
				});
			} else
				_queued = false;
			return BSEventNotifyControl::kContinue;
		}

	private:
		std::atomic<bool> _queued{ false };
	};
}

// ---------------------------------------------------------------------------------------------

bool DeepFit::MegaMouthNow()
{
	const int o = g_megaOverride.load();
	return o < 0 ? Config::get().megaMouth : o == 1;
}

bool DeepFit::MouthNow()
{
	const int o = g_mouthOverride.load();
	return o < 0 ? Config::get().mouthMorphs : o == 1;
}

void DeepFit::SetMegaOverride(int a_value)
{
	g_megaOverride = a_value;
}

void DeepFit::Install()
{
	{   // 0.4.4: is Anatomica installed? (then it tells DeepFit the schlong's shape, and DeepFit's own detection stays off)
		std::error_code ec;
		g_anatomica = std::filesystem::exists("Data/SKSE/Plugins/AnatomicaMenu.dll", ec);
		logs::info("DeepFit: Anatomica {}", g_anatomica ? "found - it tells DeepFit the schlong's shape" : "not found - DeepFit finds the schlong's shape itself");
	}
	static bool events = false;
	if (!events) {
		if (auto* src = SKSE::GetModCallbackEventSource()) {
			src->AddEventSink(ModEvents::Get());
			events = true;
		}
	}
	static bool hooked = false;
	if (!hooked) {
		hooked = true;
		AnimHook::Install();
	}
	static bool done = false;
	if (done)
		return;
	if (auto* input = BSInputDeviceManager::GetSingleton()) {
		input->AddEventSink(FrameDriver::Get());
		done = true;
		logs::info("DeepFit: frame driver registered (runs during OStim scenes too)");
	}
}

namespace
{
	std::atomic<bool> g_reloadHeads{ false };
	std::atomic<bool> g_refreshPlayer{ false };
}

void DeepFit::RequestPlayerRefresh() { g_refreshPlayer = true; }

void DeepFit::RequestHeadReload() { g_reloadHeads = true; }

// 0.4.6: the Info page - which head profile the player's head uses
std::string DeepFit::PlayerHeadText()
{
	auto* pc = RE::PlayerCharacter::GetSingleton();
	if (!pc)
		return {};
	auto it = g_receivers.find(pc->GetFormID());
	if (it == g_receivers.end() || it->second.found == 0)
		return "Your head: no DeepFit mouth bones found (yet).";
	const auto& r = it->second;
	auto&       h = Heads::All();
	std::string s = "Your head: ";
	s += (r.profile >= 0 && r.profile < static_cast<int>(h.size())) ? "head profile \"" + h[r.profile].name + "\"" : std::string("DeepFit's own bones (no head profile)");
	s += ", " + std::to_string(r.found) + " of " + std::to_string(r.nspecs()) + " bones found.";
	return s;
}

std::string DeepFit::PlayerBeardText()   // 0.5.0: the menu's Beards page
{
	auto* pc = RE::PlayerCharacter::GetSingleton();
	if (!pc)
		return {};
	auto it = g_receivers.find(pc->GetFormID());
	if (it == g_receivers.end() || it->second.found == 0)
		return "Your head: not fitted by DeepFit (yet) - beards are looked at once it is.";
	const auto& r = it->second;
	if (!r.beardHead.geo)
		return "Your face: not read yet.";
	int morphs = 0;
	for (auto& m : r.beardHead.morph)
		morphs += m.empty() ? 0 : 1;
	std::string s = "Your face: " + std::to_string(morphs) + " of DeepFit's morphs found in its .tri files.";
	if (r.beards.empty())
		return s + "\nNo beard on you.";
	for (auto& b : r.beards)
		s += "\n" + b.name + ": " + b.state + (b.culled ? " (shaved for the scene)" : "");
	return s;
}

void DeepFit::OnFrame(float delta)
{
	if (g_refreshPlayer.exchange(false)) {   // 0.4.7: the menu's "Refresh my head" - everything DeepFit did to his old head given
		                                       // back, then his head looked at again from scratch (race / head changed)
		if (auto* pc = RE::PlayerCharacter::GetSingleton()) {
			auto it = g_receivers.find(pc->GetFormID());
			if (it != g_receivers.end()) {
				auto& r = it->second;
				for (int i = 0; i < r.nspecs(); ++i)
					if (r.bones[i].valid && r.bones[i].node)
						r.bones[i].node->local.translate = r.bones[i].restLocal;
				for (auto& h : r.teethHooks) {
					auto* si = h.geo ? h.geo->GetGeometryRuntimeData().skinInstance.get() : nullptr;
					if (si && si->bones && si->skinData && h.idx < si->skinData->bones && si->bones[h.idx] == r.teethNode.get() && h.orig) {
						si->bones[h.idx] = h.orig.get();
						si->boneWorldTransforms[h.idx] = &h.orig->world;
					}
				}
				for (auto& o : r.teethHidden)
					if (o)
						o->SetAppCulled(false);
				for (auto& b : r.teethBones)
					if (b)
						b->local.scale = 1.0f;
				for (auto& h : r.teethSlots) {   // 0.5.2
					auto* si = h.geo ? h.geo->GetGeometryRuntimeData().skinInstance.get() : nullptr;
					if (si && si->bones && si->skinData && h.idx < si->skinData->bones && si->bones[h.idx] == r.teethHideNode.get() && h.orig) {
						si->bones[h.idx] = h.orig.get();
						si->boneWorldTransforms[h.idx] = &h.orig->world;
					}
				}
				for (auto& h : r.mouthSlots) {   // 0.5.4
					auto* si = h.geo ? h.geo->GetGeometryRuntimeData().skinInstance.get() : nullptr;
					if (si && si->bones && si->skinData && h.idx < si->skinData->bones && h.orig && h.idx >= 2 && h.idx < static_cast<std::uint32_t>(kThroatSlot0 + kThroatN) &&
						si->bones[h.idx] == (static_cast<int>(h.idx) >= kThroatSlot0 ? r.throatNodes[h.idx - kThroatSlot0].get() :
						                         r.mouthNodes[static_cast<int>(h.idx) == kTeethLowerSlot ? 0 : h.idx - 2].get())) {
						si->bones[h.idx] = h.orig.get();
						si->boneWorldTransforms[h.idx] = &h.orig->world;
					}
				}
				dropBeards(r);   // 0.5.0
				g_receivers.erase(it);
			}
			{
				std::lock_guard lk(g_faceLock);
				g_face.erase(pc->GetFormID());
				g_faceN = static_cast<int>(g_face.size());
			}
			logs::info("DeepFit: your head will be looked at again (menu)");
		}
	}
	if (g_reloadHeads.exchange(false)) {   // 0.4.6: the menu's "Reload head files" (here, so no head is mid-fit)
		Heads::Load();
		logs::info("DeepFit: head profiles reloaded ({})", Heads::All().size());
	}
	{   // 0.4.5: how far PPA moved each partner's schlong tip from the animation's pose (debug log, every 2 s)
		static double toldPpa = -1e9;
		if (Config::get().debugLog && g_useSnap && g_clock - toldPpa > 2.0) {
			for (auto& [id, p] : g_partners) {
				if (p.chain.empty() || !p.chain.back())
					continue;
				auto it = g_snap.find(p.chain.back().get());
				if (it == g_snap.end())
					continue;
				const float moved = len(it->second.translate - p.chain.back()->world.translate);
				if (moved > 0.5f) {
					toldPpa = g_clock;
					logs::info("DeepFit: {:08X}'s schlong tip is {:.1f} units from where the animation has it (PPA's aim) - read where PPA put it", id, moved);
				}
			}
		}
	}
	auto& cfg = Config::get();
	if (!cfg.enabled)
		return;
	if (delta <= 0.0f || delta > 0.5f)
		delta = 1.0f / 60.0f;
	g_logTimer += delta;
	g_clock += delta;
	g_tiltDeg = g_preview.on ? std::clamp(g_preview.tilt, 0.0f, 60.0f) : 0.0f;   // 0.4.2: applied in the animation update
	if (!g_preview.on)
		hideTestShaft();
	if (g_tngAsk.exchange(false))
		askTng();   // 0.4.0: TNG's sizes (again when the test page asks)

	// collect loaded actors
	std::vector<Actor*> actors;
	if (auto* pc = PlayerCharacter::GetSingleton())
		actors.push_back(pc);
	if (auto* pl = ProcessLists::GetSingleton()) {
		for (auto& h : pl->highActorHandles) {
			auto a = h.get();
			if (a && a.get() != actors[0])
				actors.push_back(a.get());
		}
	}
	for (auto& [id, p] : g_partners)
		p.skinnedThisFrame = false;

	int nRecv = 0, nShaft = 0;
	for (auto* recv : actors) {
		if (cfg.malesOnly) {   // 0.3.7: only men's mouths (a woman's SMP Head is left to its own physics and PPA)
			auto* base = recv->GetActorBase();
			if (!base || base->GetSex() != SEX::kMale) {
				static std::unordered_set<FormID> toldSkip;   // 0.5.5: say so once (a woman's mouth never opening looked like a bug)
				if (cfg.debugLog && toldSkip.insert(recv->GetFormID()).second)
					logs::info("DeepFit: {:08X} ({}) is a woman - not fitted while \"Women's heads too\" is off", recv->GetFormID(), recv->GetDisplayFullName());
				continue;
			}
		}
		auto* r = getReceiver(recv, delta);
		if (!r) {
			// 0.5.5: once a minute, who has no mouth DeepFit can fit and why (race, face mesh) - not only the player
			static std::unordered_map<FormID, double> toldNo;
			auto& t = toldNo[recv->GetFormID()];
			if (cfg.debugLog && g_clock - t > 60.0) {
				t = g_clock;
				std::string race, mesh;
				if (auto* rc = recv->GetRace(); rc && rc->GetFormEditorID())
					race = rc->GetFormEditorID();
				if (auto* base = recv->GetActorBase())
					if (auto* hp = base->GetCurrentHeadPartByType(BGSHeadPart::HeadPartType::kFace); hp && hp->GetModel())
						mesh = hp->GetModel();
				logs::info("DeepFit: {:08X} ({}) race {} head {} - no mouth bones DeepFit can fit", recv->GetFormID(), recv->GetDisplayFullName(), race, mesh);
			}
			continue;
		}
		{   // 0.4.4: a woman's head (built for DeepFit) uses the women's settings
			auto* base = recv->GetActorBase();
			r->female = base && base->GetSex() == SEX::kFemale;
		}
		++nRecv;
		const NiPoint3 headPos = r->head->world.translate;

		std::vector<NiPoint3> shaft;
		Actor*                main = nullptr;
		Partner*              mainP = nullptr;
		std::size_t           mainN = 0;
		for (auto* other : actors) {
			if (other == recv)
				continue;
			NiPoint3 dpos = other->GetPosition() - headPos;
			if (len(dpos) > cfg.maxPartnerDistance)
				continue;
			auto* p = getPartner(other, delta);
			if (!p)
				continue;
			skinPartner(*p);
			if (p->world.size() > mainN) {
				main = other;
				mainP = p;
				mainN = p->world.size();
			}
			if (cfgOf(*r).followAim && r->axesOk) {   // 0.4.3: onto his lips, the way PPA aims it
				std::vector<NiPoint3> pts = p->world;
				aimAtMouth(pts, *p, r->mouthC, cfgOf(*r));
				shaft.insert(shaft.end(), pts.begin(), pts.end());
			} else
				shaft.insert(shaft.end(), p->world.begin(), p->world.end());
		}
		// 0.4.0: the partner's race strength
		std::string raceId;
		if (main) {
			if (auto* race = main->GetRace())
				raceId = race->GetFormEditorID() ? race->GetFormEditorID() : "";
		}
		r->raceK = main ? cfg.raceStrengthFor(raceId) : 1.0f;
		// 0.4.4: his shape, found by DeepFit - only without Anatomica (it tells DeepFit itself; the two do not fight)
		if (main && mainP && cfg.detectShapes && !g_anatomica && !(r->shapeFromAnatomica && g_clock - r->shapeAt < 5.0)) {
			mainP->kindAge -= delta;
			if (mainP->kind < 0 || mainP->kindAge <= 0.0f) {
				mainP->kind = findKind(main, mainP->kindWhy);
				mainP->kindAge = 5.0f;
			}
			r->shape = mainP->kind;
			r->shapeAt = g_clock;
			r->shapeFromAnatomica = false;
		}
		nShaft += static_cast<int>(shaft.size());
		r->inScene = !shaft.empty() || (g_preview.on && recv->IsPlayerRef());
		if (g_preview.on && recv->IsPlayerRef())
			solve(*r, shaft, delta, true);   // 0.4.0: the test page
		else if (shaft.empty())
			relax(*r, delta);
		else
			solve(*r, shaft, delta);
		sendMouth(*r, delta);
		turnJaw(*r);   // 0.5.5
		soundsFor(*r, recv);   // 0.4.4
		updateTeeth(*r, recv, delta);
		hideTeeth(*r, recv, delta);
		rigMouth(*r, recv, delta);   // 0.5.4
		beardsFor(*r, recv, delta);   // 0.5.0
		if (recv->IsPlayerRef())
			updateTestShaft(*r);   // 0.4.0: the test schlong, shown
		if (r->full && main && mainP) {   // 0.4.0: what the menu shows
			char buf[512];
			std::string shapeTxt = "shape: ";
			if (r->shapeFromAnatomica && g_clock - r->shapeAt < 5.0)
				shapeTxt += std::string(r->shape == 3 ? "flared" : r->shape == 4 ? "knotted" : r->shape == 6 ? "ridged" : r->shape == 7 ? "segmented" : "as Anatomica says") + " (from Anatomica)";
			else if (mainP->kind >= 0)
				shapeTxt += std::string(kindName(mainP->kind)) + " (" + mainP->kindWhy + ")";
			else
				shapeTxt += "as read from his mesh";
			std::snprintf(buf, sizeof(buf), "%s with %s (%s): %s, %s; %s; tip %.1f deep, race strength x%.2f",
				recv->GetDisplayFullName(), main->GetDisplayFullName(), raceId.empty() ? "?" : raceId.c_str(), mainP->source.c_str(),
				mainP->method == 1 ? "shaft built round his bones" : "his mesh read", shapeTxt.c_str(), r->depthPrev, r->raceK);
			std::lock_guard lk(g_statusLock);
			g_status.text = buf;
			g_status.race = raceId;
			g_status.at = g_clock;
		}
	}
	static float heartbeat = 0.0f;
	heartbeat += delta;
	if (cfg.debugLog && heartbeat > 5.0f) {
		heartbeat = 0.0f;
		logs::info("DeepFit: alive - {} actors, {} with mouth bones, {} shaft verts in range", actors.size(), nRecv, nShaft);
		static double namesAt = -1e9;   // 0.5.5: and who they are, once a minute
		if (g_clock - namesAt > 60.0) {
			namesAt = g_clock;
			std::string who;
			for (auto* a : actors)
				who += std::format("{}{:08X} {}", who.empty() ? "" : ", ", a->GetFormID(), a->GetDisplayFullName() ? a->GetDisplayFullName() : "?");
			logs::info("DeepFit: actors looked at: {}", who);
		}
	}
	if (g_logTimer > 1.0f)
		g_logTimer = 0.0f;
}

void DeepFit::CaptureKey(bool a_on) { g_capture = a_on; }
bool DeepFit::CapturingKey() { return g_capture; }
const char* DeepFit::KeyText(int a_code, int a_mods)
{
	static std::string out;
	if (a_code <= 0)
		return "none";
	static const char* kNames[] = {
		"?", "Esc", "1", "2", "3", "4", "5", "6", "7", "8", "9", "0", "-", "=", "Backspace", "Tab",
		"Q", "W", "E", "R", "T", "Y", "U", "I", "O", "P", "[", "]", "Enter", "Ctrl", "A", "S",
		"D", "F", "G", "H", "J", "K", "L", ";", "'", "`", "Shift", "\\", "Z", "X", "C", "V",
		"B", "N", "M", ",", ".", "/", "Shift", "Num *", "Alt", "Space", "Caps Lock", "F1", "F2", "F3", "F4", "F5",
		"F6", "F7", "F8", "F9", "F10", "Num Lock", "Scroll Lock", "Num 7", "Num 8", "Num 9", "Num -", "Num 4", "Num 5", "Num 6", "Num +", "Num 1",
		"Num 2", "Num 3", "Num 0", "Num ."
	};
	std::string k;
	if (a_code < static_cast<int>(std::size(kNames))) k = kNames[a_code];
	else if (a_code == 87) k = "F11";
	else if (a_code == 88) k = "F12";
	else if (a_code == 199) k = "Home";
	else if (a_code == 200) k = "Up";
	else if (a_code == 201) k = "Page Up";
	else if (a_code == 203) k = "Left";
	else if (a_code == 205) k = "Right";
	else if (a_code == 207) k = "End";
	else if (a_code == 208) k = "Down";
	else if (a_code == 209) k = "Page Down";
	else if (a_code == 210) k = "Insert";
	else if (a_code == 211) k = "Delete";
	else k = "key " + std::to_string(a_code);
	out.clear();
	if (a_mods & 2) out += "Ctrl+";
	if (a_mods & 1) out += "Shift+";
	if (a_mods & 4) out += "Alt+";
	out += k;
	return out.c_str();
}

DeepFit::Preview& DeepFit::GetPreview() { return g_preview; }

DeepFit::TngSizes DeepFit::GetTngSizes()
{
	std::lock_guard lk(g_tngLock);
	return g_tng;
}

void DeepFit::AskTng() { g_tngAsk = true; }

int DeepFit::SoundCount(const char* a_kind)
{
	std::lock_guard lk(g_soundLock);
	return static_cast<int>(lookSounds(a_kind).files.size());
}

void DeepFit::RescanSounds()
{
	std::lock_guard lk(g_soundLock);
	g_sounds.clear();
}

bool DeepFit::AnatomicaFound() { return g_anatomica; }

std::string DeepFit::StatusText(std::string* a_race, float* a_ago)
{
	std::lock_guard lk(g_statusLock);
	if (a_race) *a_race = g_status.race;
	if (a_ago) *a_ago = static_cast<float>(g_clock - g_status.at);
	return g_status.text;
}
