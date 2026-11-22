#include <unordered_set>
#include <functional>
#include <cctype>
#include <mutex>
#include "DeepFit.h"
#include "Config.h"
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
		Cheek   // 0.4.0
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
	};
	constexpr int kUpperCenter = 0;
	constexpr int kBottomCenter = 3;
	constexpr int kNumSpecs = static_cast<int>(sizeof(kSpecs) / sizeof(kSpecs[0]));

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
	constexpr int         kMorphs = kKnob0 + kBands;
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
		"DF_ThroatKnob6", "DF_ThroatKnob7", "DF_ThroatKnob8", "DF_ThroatKnob9", "DF_ThroatKnob10" };

	struct Receiver
	{
		NiAVObject*           root = nullptr;  // identity of the 3D we cached from
		NiPointer<NiAVObject> head;
		NiPointer<NiAVObject> hmag;          // 0.1.3: "NPC Head MagicNode [Hmag]" - in front of his face
		NiPoint3              fx{ 1, 0, 0 }, fy{ 0, 1, 0 }, fz{ 0, 0, 1 };   // 0.1.3: his face's right / forward / up, this frame
		bool                  toldFrame = false;
		DrivenBone            bones[kNumSpecs];
		ActorHandle           actor;                 // 0.2.0: whose face the morphs go on
		float                 morph[kMorphs] = {};   // 0.2.0: eased mouth morph values (0-100)
		int                   sent[kMorphs];   // set to -1 (never sent) in the constructor
		Receiver() { std::fill(std::begin(sent), std::end(sent), -1); }
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
		NiPointer<NiNode>      teethNode;
		float                  teethK = 0.0f, teethScan = 0.0f;
		bool                   teethTold = false;
	};

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
		float                   radius = 1.6f;
		int                     method = -1;   // 0 the mesh, 1 the bone line (last frame), for the log
		float                   meshOff = 0.0f;
		bool                    anyMesh = false;   // 0.4.0: nothing recognisable - his whole mesh is read (only what reaches the lips counts)
		std::string             source;            // 0.4.0: how he was read, for the menu
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
	double                               g_clock = 0.0;   // 0.4.0: seconds since the game started (for gulps / gags)
	DeepFit::Preview                     g_preview;        // 0.4.0: the menu's test page
	std::unordered_map<FormID, Partner>  g_partners;
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
	int matchSpec(const char* name)
	{
		if (!name)
			return -1;
		std::string_view n(name);
		for (int i = 0; i < kNumSpecs; ++i) {
			std::string_view suf(kSpecs[i].suffix);
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

	Receiver* getReceiver(Actor* actor, float delta)
	{
		auto* root = actor->Get3D(false);
		if (!root)
			return nullptr;
		auto& r = g_receivers[actor->GetFormID()];
		if (r.root == root && r.found > 0) {
			// 0.3.2: the face can be rebuilt under the same body (a face paint / overlay change, an expression mod): the lip
			// bones we kept are then the old, detached ones - frozen where the face once was, ~500 units off in the log -
			// so the mouth never met the shaft. Still on his head, near it? Then keep them, else look them up again.
			auto& up = r.bones[kUpperCenter];
			const bool live = r.head && r.head->parent && up.valid && up.node && up.node->parent && up.node->parent == up.parent.get() &&
			                  len(up.node->world.translate - r.head->world.translate) < 40.0f;
			if (live)
				return &r;
			if (Config::get().debugLog)
				logs::info("DeepFit: {:08X}'s face was rebuilt - finding the mouth bones again", actor->GetFormID());
			r.found = 0;
			r.checked = false;
		}
		if (r.root == root && r.found == 0 && r.checked) {
			// the facegen head attaches a little after the body 3D: keep looking every 2 s
			r.retry -= delta;
			if (r.retry > 0.0f)
				return nullptr;
		}

		// (re)build
		r = Receiver{};
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
		traverse(root, [&](NiAVObject* o) {
			int i = matchSpec(o->name.c_str());
			if (i < 0 || !o->parent)
				return;
			auto& b = r.bones[i];
			if (b.valid)
				return;  // first match wins
			b.node = NiPointer<NiAVObject>(o);
			b.parent = NiPointer<NiNode>(o->parent);
			b.restLocal = o->local.translate;
			b.cur = 0.0f;
			b.valid = true;
			++r.found;
		});
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

			ShaftShape shape;
			shape.geom = NiPointer<BSGeometry>(geom);
			for (std::uint32_t j = 0; j < sp->vertexCount; ++j) {
				const std::uint8_t*  v = data + static_cast<std::size_t>(j) * vSize;
				const std::uint16_t* hw = reinterpret_cast<const std::uint16_t*>(v + skinOff);
				const std::uint8_t*  bi = reinterpret_cast<const std::uint8_t*>(hw + 4);
				float                w[4];
				float                genW = 0.0f, tot = 0.0f;
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
					sv.bone[k] = bi[k];
					sv.weight[k] = w[k] / tot;
				}
				shape.verts.push_back(sv);
			}
			if (!shape.verts.empty())
				p.shapes.push_back(std::move(shape));
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
			if (n) b.push_back(n->world.translate);
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
				xf[i] = b->world * sd->boneData[i].skinToBone;
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
					const NiPoint3 a = p.chain[k]->world.translate, b = p.chain[k + 1]->world.translate, ab = b - a;
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

	void solve(Receiver& r, const std::vector<NiPoint3>& shaft, float delta, bool preview = false)
	{
		auto& cfg = Config::get();
		r.full = false;

		// rest world position of every driven bone
		NiPoint3 rest[kNumSpecs];
		for (int i = 0; i < kNumSpecs; ++i) {
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
			if (cfg.debugLog && !r.toldFrame) {
				r.toldFrame = true;
				const NiPoint3 hx = norm(H.rotate * NiPoint3(1, 0, 0)), hy = norm(H.rotate * NiPoint3(0, 1, 0)), hz = norm(H.rotate * NiPoint3(0, 0, 1));
				logs::info("DeepFit: face axes from the mouth (right/forward/up) against the head bone's x/y/z: right {:.2f}/{:.2f}/{:.2f}, forward {:.2f}/{:.2f}/{:.2f}, up {:.2f}/{:.2f}/{:.2f}",
					dot(X, hx), dot(X, hy), dot(X, hz), dot(Y, hx), dot(Y, hy), dot(Y, hz), dot(Z, hx), dot(Z, hy), dot(Z, hz));
			}
		}
		const NiPoint3 X = r.fx, Y = r.fy, Z = r.fz;
		const NiPoint3 C = (rest[kUpperCenter] + rest[kBottomCenter]) * 0.5f;

		// shaft vertices in head space, relative to the mouth centre
		std::vector<NiPoint3> hs;
		hs.reserve(shaft.size());
		for (auto& v : shaft) {
			NiPoint3 p = v - C;
			hs.emplace_back(dot(p, X), dot(p, Y), dot(p, Z));
		}
		// 0.4.0: the test page - a made-up schlong straight into his mouth (no scene needed), with the chosen shape
		if (preview) {
			hs.clear();
			const auto& pv = g_preview;
			float D = std::max(pv.depth, 0.5f);
			if (pv.strokes)
				D *= 0.78f + 0.22f * std::sin(static_cast<float>(g_clock) * 3.7699f);   // in and out, 0.6 a second
			const float T = std::max(pv.thick, 0.2f), W = std::max(pv.width, 0.2f);
			for (float sIn = -3.0f; sIn <= D; sIn += 0.25f) {
				float k = 1.0f;
				if (pv.shape == 1) {   // horse: a medial ring, and the flared head
					k += 0.3f * std::exp(-std::pow((sIn - D * 0.45f) / 0.35f, 2.0f));
					if (sIn > D - 2.2f) k = std::max(k, 1.45f);
				} else if (pv.shape == 2)   // knot: a bulb near the base
					k += 0.85f * std::exp(-std::pow((sIn - std::min(2.0f, D * 0.3f)) / 0.9f, 2.0f));
				const float tipK = std::sqrt(std::clamp((D - sIn) / 0.8f, 0.0f, 1.0f));   // the rounded tip
				const float rz = T * k * std::max(tipK, 0.15f), rx = W * k * std::max(tipK, 0.15f);
				for (int a = 0; a < 16; ++a) {
					const float ang = a * 0.3926991f;
					hs.emplace_back(rx * std::cos(ang), -sIn, rz * std::sin(ang));
				}
			}
			r.shape = pv.shape == 3 ? 6 : pv.shape == 4 ? 7 : 0;
			r.shapeAt = g_clock;
		}

		float target[kNumSpecs] = {};
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
		float comp[kNumSpecs] = {};   // how far the morphs already moved each lip (the bones then only make up the rest)
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
			const bool  through = zTop > zBot;
			const float height = through ? zTop - zBot : 0.0f;
			const float width = through ? xR - xL : 0.0f;
			const float restWidth = (r.bones[6].valid && r.bones[7].valid) ? len(rest[7] - rest[6]) : 4.0f;
			float want[kMorphs] = {};
			if (through || (inMouth && DeepFit::MegaMouthNow())) {   // 0.3.1: Mega mouth opens fully as soon as it is in his mouth
				const float open = height + cfg.mouthExtra;
				want[0] = DeepFit::MegaMouthNow() ? cfg.morphMax : open / std::max(cfg.jawUnits, 0.1f) * 100.0f;
				want[1] = DeepFit::MegaMouthNow() ? cfg.morphMax * cfg.lipRound : open * 0.5f / std::max(cfg.lipUnits, 0.1f) * 100.0f * cfg.lipRound;
				// 0.3.8: the corners go out to the sides too - to the shaft's width (plus MouthExtra) and a share of how far
				// the jaw is open - so the mouth opens wide round it instead of pushing the lips forward into a pucker
				{
					const float out = std::max(0.0f, 0.5f * (width + cfg.mouthExtra) - 0.5f * restWidth) + cfg.cornerOpen * open;
					want[kMouthWide] = DeepFit::MegaMouthNow() ? 100.0f * cfg.mouthWideMax : 100.0f * std::clamp(out * cfg.mouthWideGain, 0.0f, cfg.mouthWideMax);
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
				float raw[kBands], reachK[kBands];
				for (int k = 0; k < kBands; ++k) {
					const float s = cfg.throatFrom + (k + 0.5f) * cfg.throatBand;
					const float lo = s - 0.25f * cfg.throatBand, hi = s + 0.25f * cfg.throatBand;
					const float rz = std::max(halfH(lo), halfH(hi)), rx = std::max(halfW(lo), halfW(hi));
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
			// the lips drag with it: rolled out as he pulls back, tucked in as he pushes in
			r.dragWant = r.full ? std::clamp(-r.vel / std::max(cfg.lipDragSpeed, 1.0f), -1.0f, 1.0f) * cfg.lipDrag : 0.0f;
			const float km = 1.0f - std::exp(-delta * 10.0f);
			for (int m = 0; m < kMorphs; ++m) {
				want[m] = std::clamp(want[m], 0.0f, cfg.morphMax);
				r.morph[m] += (want[m] - r.morph[m]) * km;
			}
			r.teeth = cfg.teethOpen && (through || inMouth);
			// what the morphs did to each lip (their size at 100, from the SMP Head tri): the bones only make up the rest
			const float jaw = r.morph[0] / 100.0f * cfg.jawUnits, round = r.morph[1] / 100.0f * cfg.lipUnits, widen = r.morph[2] / 100.0f * cfg.widenUnits;
			for (int i = 0; i < kNumSpecs; ++i) {
				if (kSpecs[i].kind == Kind::Lip)
					comp[i] = kSpecs[i].dir.z > 0 ? round * 0.5f : jaw * 0.85f + round * 0.5f;
				else if (kSpecs[i].kind == Kind::Corner)
					comp[i] = widen + r.morph[kMouthWide] / 100.0f;   // 0.3.8: DF_MouthWide moves each corner 1 unit at 100
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
		for (int i = 0; i < kNumSpecs; ++i) {
			auto& b = r.bones[i];
			if (!b.valid)
				continue;
			const auto& spec = kSpecs[i];
			NiPoint3    rel = rest[i] - C;
			NiPoint3    rh(dot(rel, X), dot(rel, Y), dot(rel, Z));

			if (spec.kind == Kind::Lip || spec.kind == Kind::Corner) {
				NiPoint3 d = norm(spec.dir);
				NiPoint3 e(d.z, 0.0f, -d.x);  // across the lip, in the mouth plane
				float    sRest = rh.x * d.x + rh.z * d.z;
				float    sMax = -1e9f;
				const float mxLip = spec.kind == Kind::Corner ? cfg.cornerMax : (spec.dir.z > 0 ? cfg.upperLipMax : cfg.lowerLipMax);
				for (auto& p : hs) {
					if (std::fabs(p.y - rh.y) > cfg.lipDepthSlab)
						continue;
					if (std::fabs(p.x * e.x + p.z * e.z) > cfg.lipLaneWidth)
						continue;
					float s = p.x * d.x + p.z * d.z;
					if (s - sRest > mxLip * 2.0f + 1.0f)
						continue;   // 0.1.4: far past what this lip can reach (the shaft's base, the balls) - not round the lip
					if (s > sMax)
						sMax = s;
				}
				float mx = mxLip;
				if (DeepFit::MouthNow()) {   // 0.2.0: the morphs opened the mouth; the bones fit the lips to the shaft (out, or back in round a slim one)
					target[i] = sMax > -1e8f ? std::clamp(sMax + cfg.lipMargin + cfg.stretch - sRest - comp[i], DeepFit::MegaMouthNow() ? 0.0f : -cfg.lipCloseMax, mx) : 0.0f;
					// 0.3.3: the lip-round morph peaks the middle of the upper lip up towards the nose - pull it back down
					if (spec.kind == Kind::Lip && spec.dir.z > 0.0f && (sMax > -1e8f || inMouth) && cfg.upperLipDown > 0.0f)
						target[i] -= cfg.upperLipDown * (i == kUpperCenter ? 1.0f : 0.6f);
				} else {
					target[i] = sMax > -1e8f ? std::clamp(sMax + cfg.lipMargin + cfg.stretch - sRest, 0.0f, mx) : 0.0f;
					if (inMouth)
						target[i] = std::min(std::max(target[i], spec.kind == Kind::Corner ? cfg.minCorner : cfg.minOpen), mx);
				}
			} else if (spec.kind == Kind::Cheek) {
				// 0.4.0: a shaft pushed to his side of the mouth (at the cheek's depth) bulges that cheek out
				if (cfg.cheekGain <= 0.0f)
					continue;
				const float sgn = spec.dir.x < 0.0f ? -1.0f : 1.0f;
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
		// nose base follows the upper lip centre
		if (r.bones[8].valid)
			target[8] = std::clamp(target[kUpperCenter] * cfg.noseFollow, 0.0f, cfg.noseMax);

		// smooth + apply
		const float k = 1.0f - std::exp(-delta * cfg.smoothSpeed);
		r.drag += (r.dragWant - r.drag) * (1.0f - std::exp(-delta * 8.0f));
		for (int i = 0; i < kNumSpecs; ++i) {
			auto& b = r.bones[i];
			if (!b.valid)
				continue;
			b.cur += (target[i] - b.cur) * k;
			if (std::fabs(b.cur) < 1e-4f)
				b.cur = 0.0f;
			NiPoint3 d = norm(kSpecs[i].dir);
			NiPoint3 offWorld = (X * d.x + Y * d.y + Z * d.z) * b.cur;
			if (kSpecs[i].kind == Kind::Lip || kSpecs[i].kind == Kind::Corner)   // 0.4.0: the lips drag with the stroke
				offWorld = offWorld + Y * (r.drag * (kSpecs[i].kind == Kind::Corner ? 0.4f : 1.0f));
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
		auto& cfg = Config::get();
		r.full = false;
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
		for (int i = 0; i < kNumSpecs; ++i) {
			auto& b = r.bones[i];
			if (!b.valid || (b.cur == 0.0f && r.drag == 0.0f))
				continue;
			b.cur *= (1.0f - k);
			if (std::fabs(b.cur) < 1e-4f)
				b.cur = 0.0f;
			if (std::fabs(r.drag) < 1e-4f)
				r.drag = 0.0f;
			NiPoint3 d = norm(kSpecs[i].dir);
			NiPoint3 offWorld = (r.fx * d.x + r.fy * d.y + r.fz * d.z) * b.cur;   // 0.1.3: the face's own axes
			if (kSpecs[i].kind == Kind::Lip || kSpecs[i].kind == Kind::Corner)
				offWorld = offWorld + r.fy * (r.drag * (kSpecs[i].kind == Kind::Corner ? 0.4f : 1.0f));
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
					}
				}
			} else if (name == "DeepFit_Mouth") {
				g_mouthOverride = v;
				logs::info("DeepFit: opening the mouth {} (mod event)", v < 0 ? "back to the setting" : (v ? "on" : "off"));
			}
			return BSEventNotifyControl::kContinue;
		}
	};

	// 0.2.0: hand the mouth morphs to MFEE (its Papyrus function, on the game thread we run on) and the teeth modifier
	// to the face; only when a value has really changed, at most 20 times a second
	void sendMouth(Receiver& r, float delta)
	{
		auto& cfg = Config::get();
		if (!DeepFit::MouthNow())
			return;
		auto act = r.actor.get();
		if (!act)
			return;
		if (auto* fg = act->GetFaceGenAnimationData(); fg && cfg.teethOpen && (r.teeth || r.teethSent)) {   // 0.2.1: only while in use, and once to close
			fg->modifierKeyFrame.SetValue(12, r.teeth ? 1.0f : 0.0f);
			r.teethSent = r.teeth;
		}
		// 0.4.0: his face reacts - eyes close and brows lift the deeper it goes, eyes clench and brows knit in a gag
		if (auto* fg = act->GetFaceGenAnimationData(); fg && fg->modifierKeyFrame.values && fg->modifierKeyFrame.count >= 8) {
			float want[8] = {};
			if (cfg.faceReact) {
				const double tk = g_clock - r.gagAt;
				const float  gag = (cfg.gagTime > 0.0f && tk >= 0.0 && tk < cfg.gagTime) ? std::sin(3.14159f * static_cast<float>(tk / cfg.gagTime)) * std::min(r.gagK, 1.0f) : 0.0f;
				const float  eyes = std::clamp(cfg.faceEyes * r.depthK + cfg.faceGag * gag, 0.0f, 1.0f);
				want[0] = want[1] = eyes;                                                       // blink left / right
				want[4] = want[5] = std::clamp(cfg.faceGag * gag * 0.8f, 0.0f, 1.0f);           // brows in
				want[6] = want[7] = std::clamp(cfg.faceBrows * r.depthK * (1.0f - gag), 0.0f, 1.0f);   // brows up
			}
			bool any = false;
			for (float w : want)
				any |= w > 0.01f;
			if (any || r.faceOn) {
				auto* vals = fg->modifierKeyFrame.values;
				for (std::uint32_t i = 0; i < 8; ++i) {
					if (i == 2 || i == 3)
						continue;
					const float cur = vals[i];
					if (std::fabs(cur - r.faceSent[i]) > 1e-3f)
						r.faceBase[i] = cur;   // someone else (OStim's expressions, blinking) set it: that is what we go over
					const float out = any ? std::max(r.faceBase[i], want[i]) : r.faceBase[i];
					fg->modifierKeyFrame.SetValue(i, out);
					r.faceSent[i] = out;
				}
				r.faceOn = any;
			}
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
			const int v = std::max(0, static_cast<int>(std::lround(r.morph[m] + extra[m])));
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
	}

	// 0.4.0: the teeth moved a little down / back. Their weights on his head bone are hung on a node of DeepFit's own under
	// it ("DeepFit Teeth"), so they follow his head exactly and only that node is moved. The game rebuilding the face puts
	// them back on the head bone: they are checked every second and hung on it again. Mode 1 while the mouth is full,
	// mode 2 always (only the actors DeepFit fits, and only when chosen).
	void updateTeeth(Receiver& r, Actor* act, float delta)
	{
		auto& cfg = Config::get();
		const bool wantOn = cfg.teethMove == 2 || (cfg.teethMove == 1 && r.full);
		const float to = wantOn ? 1.0f : 0.0f;
		r.teethK += (to - r.teethK) * (1.0f - std::exp(-delta * 6.0f));
		if (std::fabs(r.teethK - to) < 1e-3f)
			r.teethK = to;
		const bool moving = cfg.teethMove > 0 && (std::fabs(cfg.teethDown) > 1e-3f || std::fabs(cfg.teethBack) > 1e-3f);
		if (!moving || (r.teethK == 0.0f && !wantOn)) {
			if (!r.teethHooks.empty() && (!moving || cfg.teethMove == 0)) {   // switched off: back on his head bone
				for (auto& h : r.teethHooks) {
					auto* si = h.geo ? h.geo->GetGeometryRuntimeData().skinInstance.get() : nullptr;
					if (si && si->bones && h.idx < si->skinData->bones && si->bones[h.idx] == r.teethNode.get() && h.orig) {
						si->bones[h.idx] = h.orig.get();
						si->boneWorldTransforms[h.idx] = &h.orig->world;
					}
				}
				r.teethHooks.clear();
			}
			if (r.teethNode)
				r.teethNode->local.translate = NiPoint3(0, 0, 0);
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
			headNode->AttachChild(r.teethNode.get(), true);
			r.teethScan = 0.0f;
		}
		r.teethScan -= delta;
		if (r.teethScan <= 0.0f) {
			r.teethScan = 1.0f;
			std::vector<std::string> parts;
			{
				std::string cur;
				for (char c : cfg.teethParts + ",") {
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
					if (b == r.teethNode.get())
						continue;   // still ours
					if (b != r.head.get() && std::string_view(b->name.c_str() ? b->name.c_str() : "") != "NPC Head [Head]")
						continue;   // only what hangs on his head bone
					r.teethHooks.push_back({ NiPointer<BSGeometry>(geo), i, NiPointer<NiAVObject>(b) });
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
		const NiPoint3 offWorld = (r.fz * -cfg.teethDown + r.fy * -cfg.teethBack) * r.teethK;
		const NiTransform& H = r.head->world;
		NiPoint3 off = H.rotate.Transpose() * offWorld;
		if (H.scale > 1e-6f)
			off = off * (1.0f / H.scale);
		r.teethNode->local.translate = off;
		r.teethNode->world = H * r.teethNode->local;
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
	static bool events = false;
	if (!events) {
		if (auto* src = SKSE::GetModCallbackEventSource()) {
			src->AddEventSink(ModEvents::Get());
			events = true;
		}
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

void DeepFit::OnFrame(float delta)
{
	auto& cfg = Config::get();
	if (!cfg.enabled)
		return;
	if (delta <= 0.0f || delta > 0.5f)
		delta = 1.0f / 60.0f;
	g_logTimer += delta;
	g_clock += delta;

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
			if (!base || base->GetSex() != SEX::kMale)
				continue;
		}
		auto* r = getReceiver(recv, delta);
		if (!r)
			continue;
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
			shaft.insert(shaft.end(), p->world.begin(), p->world.end());
		}
		// 0.4.0: the partner's race strength
		std::string raceId;
		if (main) {
			if (auto* race = main->GetRace())
				raceId = race->GetFormEditorID() ? race->GetFormEditorID() : "";
		}
		r->raceK = main ? cfg.raceStrengthFor(raceId) : 1.0f;
		nShaft += static_cast<int>(shaft.size());
		if (g_preview.on && recv->IsPlayerRef())
			solve(*r, shaft, delta, true);   // 0.4.0: the test page
		else if (shaft.empty())
			relax(*r, delta);
		else
			solve(*r, shaft, delta);
		sendMouth(*r, delta);
		updateTeeth(*r, recv, delta);
		if (r->full && main && mainP) {   // 0.4.0: what the menu shows
			char buf[512];
			std::snprintf(buf, sizeof(buf), "%s with %s (%s): %s, %s; tip %.1f deep, race strength x%.2f",
				recv->GetDisplayFullName(), main->GetDisplayFullName(), raceId.empty() ? "?" : raceId.c_str(), mainP->source.c_str(),
				mainP->method == 1 ? "shaft built round his bones" : "his mesh read", r->depthPrev, r->raceK);
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

std::string DeepFit::StatusText(std::string* a_race, float* a_ago)
{
	std::lock_guard lk(g_statusLock);
	if (a_race) *a_race = g_status.race;
	if (a_ago) *a_ago = static_cast<float>(g_clock - g_status.at);
	return g_status.text;
}
