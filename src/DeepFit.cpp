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
		Throat
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
	constexpr int         kMorphs = kWide0 + kBands;
	constexpr const char* kMorphNames[kMorphs] = { "PPA_JawWide", "PPA_LipStretch", "PPA_LipWiden", "PPA_TH1", "PPA_TH2", "PPA_TH3",
		"DF_ThroatFill1", "DF_ThroatFill2", "DF_ThroatFill3", "DF_ThroatFill4", "DF_ThroatFill5",
		"DF_ThroatFill6", "DF_ThroatFill7", "DF_ThroatFill8", "DF_ThroatFill9", "DF_ThroatFill10",
		"DF_Throat1", "DF_Throat2", "DF_Throat3", "DF_Throat4", "DF_Throat5",
		"DF_Throat6", "DF_Throat7", "DF_Throat8", "DF_Throat9", "DF_Throat10",
		"DF_ThroatWide1", "DF_ThroatWide2", "DF_ThroatWide3", "DF_ThroatWide4", "DF_ThroatWide5",
		"DF_ThroatWide6", "DF_ThroatWide7", "DF_ThroatWide8", "DF_ThroatWide9", "DF_ThroatWide10" };

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
	};

	// his schlong's bones, base to tip
	std::vector<NiPointer<NiAVObject>> findChain(NiAVObject* root)
	{
		static const std::vector<std::vector<const char*>> kChains{
			{ "NPC Genitals01 [Gen01]", "NPC Genitals02 [Gen02]", "NPC Genitals03 [Gen03]", "NPC Genitals04 [Gen04]", "NPC Genitals05 [Gen05]", "NPC Genitals06 [Gen06]" },
			{ "WWD 2", "WWD 3", "WWD 4", "WWD 5", "WWD 6", "WWD 7", "WWD 8", "WWD 9" },
			{ "VLDick02", "VLDick03", "VLDick04", "VLDick05", "VLDick06" },
			{ "BearD 4", "BearD 5", "BearD 6", "BearD 7", "BearD 8", "BearD 9" }
		};
		std::vector<NiPointer<NiAVObject>> best;
		for (auto& names : kChains) {   // the beast chains win over the human bones a beast skeleton also carries (unused)
			std::vector<NiPointer<NiAVObject>> c;
			for (auto* n : names)
				if (auto* o = root->GetObjectByName(n))
					c.emplace_back(o);
			if (c.size() >= 3 && (best.empty() || names[0][0] != 'N'))
				best = std::move(c);
		}
		return best;
	}

	std::atomic<bool>                    g_capture{ false };   // 0.3.1: the menu waits for the Mega mouth key
	int                                  g_mods = 0;           // Shift 1 / Ctrl 2 / Alt 4 held now
	std::unordered_map<FormID, Receiver> g_receivers;
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

	bool isGenitalBone(const char* name)
	{
		if (!name)
			return false;
		std::string_view n(name);
		// "NPC Genitals01 [Gen01]" ... "NPC Genitals06 [Gen06]"; 0.3.1: a werewolf's (WWD 2-9), a Vampire Lord's (VLDick02-06)
		// and a werebear's (BearD 4-9) hang on their own bones
		if (n.find("Scrotum") != std::string_view::npos)
			return false;
		return n.find("NPC Genitals0") != std::string_view::npos || n.starts_with("WWD ") || n.starts_with("VLDick") || n.starts_with("BearD ");
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

	void solve(Receiver& r, const std::vector<NiPoint3>& shaft, float delta)
	{
		auto& cfg = Config::get();

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
				want[1] = DeepFit::MegaMouthNow() ? cfg.morphMax : open * 0.5f / std::max(cfg.lipUnits, 0.1f) * 100.0f;
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
				for (auto& p : hs) {
					if (p.x * p.x + p.z * p.z > 64.0f)
						continue;   // not in line with his mouth (the balls, his partner's belly)
					const int b = static_cast<int>(std::floor(-p.y * 2.0f));
					if (b < 0 || b >= kBins)
						continue;
					zLo[b] = std::min(zLo[b], p.z); zHi[b] = std::max(zHi[b], p.z);
					xLo[b] = std::min(xLo[b], p.x); xHi[b] = std::max(xHi[b], p.x);
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
				float raw[kBands];
				for (int k = 0; k < kBands; ++k) {
					const float s = cfg.throatFrom + (k + 0.5f) * cfg.throatBand;
					const float lo = s - 0.25f * cfg.throatBand, hi = s + 0.25f * cfg.throatBand;
					const float rz = std::max(halfH(lo), halfH(hi)), rx = std::max(halfW(lo), halfW(hi));
					// eased in over the last two bands before the tip: a rounded end, not a cut-off
					float reach = std::clamp((tip - (s - 0.5f * cfg.throatBand)) / (2.0f * cfg.throatBand), 0.0f, 1.0f);
					reach = reach * reach * (3.0f - 2.0f * reach);
					const bool here = rz > 0.0f && reach > 0.0f;
					want[kFill0 + k] = here ? 100.0f * cfg.throatFill * reach : 0.0f;
					raw[k] = here ? std::clamp((rz - cfg.throatRest) * cfg.throatShapeGain, 0.0f, cfg.throatShapeMax) : 0.0f;
					// wider than it is tall: the bulge spreads sideways (not further out)
					want[kWide0 + k] = here ? 100.0f * reach * std::clamp((rx - std::max(rz, cfg.throatRest)) / std::max(cfg.throatWideUnits, 0.1f), 0.0f, 1.5f) : 0.0f;
				}
				for (int k = 0; k < kBands; ++k)   // a touch of blur along it, so a sudden step in the shaft is not a shelf
					want[kBand0 + k] = 100.0f * (0.15f * raw[std::max(k - 1, 0)] + 0.7f * raw[k] + 0.15f * raw[std::min(k + 1, kBands - 1)]);
			}
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
					comp[i] = widen;
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
		for (int i = 0; i < kNumSpecs; ++i) {
			auto& b = r.bones[i];
			if (!b.valid)
				continue;
			b.cur += (target[i] - b.cur) * k;
			if (std::fabs(b.cur) < 1e-4f)
				b.cur = 0.0f;
			NiPoint3 d = norm(kSpecs[i].dir);
			NiPoint3 offWorld = (X * d.x + Y * d.y + Z * d.z) * b.cur;
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
		{   // 0.2.0: no shaft: the mouth morphs ease back to nothing
			const float km = 1.0f - std::exp(-delta * 6.0f);
			for (int m = 0; m < kMorphs; ++m)
				r.morph[m] -= r.morph[m] * km;
			r.teeth = false;
		}
		// no shaft nearby: ease every bone back to rest
		const float k = 1.0f - std::exp(-delta * Config::get().smoothSpeed);
		for (int i = 0; i < kNumSpecs; ++i) {
			auto& b = r.bones[i];
			if (!b.valid || b.cur == 0.0f)
				continue;
			b.cur *= (1.0f - k);
			if (std::fabs(b.cur) < 1e-4f)
				b.cur = 0.0f;
			NiPoint3 d = norm(kSpecs[i].dir);
			NiPoint3 offWorld = (r.fx * d.x + r.fy * d.y + r.fz * d.z) * b.cur;   // 0.1.3: the face's own axes
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
		r.sendTimer -= delta;
		if (r.sendTimer > 0.0f)
			return;
		r.sendTimer = 0.05f;
		auto* vm = BSScript::Internal::VirtualMachine::GetSingleton();
		if (!vm)
			return;
		for (int m = 0; m < kMorphs; ++m) {
			const int v = static_cast<int>(std::lround(r.morph[m]));
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
		auto* r = getReceiver(recv, delta);
		if (!r)
			continue;
		++nRecv;
		const NiPoint3 headPos = r->head->world.translate;

		std::vector<NiPoint3> shaft;
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
			shaft.insert(shaft.end(), p->world.begin(), p->world.end());
		}
		nShaft += static_cast<int>(shaft.size());
		if (shaft.empty())
			relax(*r, delta);
		else
			solve(*r, shaft, delta);
		sendMouth(*r, delta);
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
