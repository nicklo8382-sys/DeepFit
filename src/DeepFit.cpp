#include "DeepFit.h"
#include "Config.h"

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
	constexpr int         kMorphs = 6;
	constexpr const char* kMorphNames[kMorphs] = { "PPA_JawWide", "PPA_LipStretch", "PPA_LipWiden", "PPA_TH1", "PPA_TH2", "PPA_TH3" };

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
		int                   sent[kMorphs] = { -1, -1, -1, -1, -1, -1 };
		float                 sendTimer = 0.0f;
		bool                  teeth = false;
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
	};

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
		// "NPC Genitals01 [Gen01]" ... "NPC Genitals06 [Gen06]"
		return n.find("NPC Genitals0") != std::string_view::npos && n.find("Scrotum") == std::string_view::npos;
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
		if (r.root == root && r.found > 0)
			return &r;
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
		return p.shapes.empty() ? nullptr : &p;
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
		if (cfg.mouthMorphs) {
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
			if (through) {
				const float open = height + cfg.mouthExtra;
				want[0] = cfg.megaMouth ? cfg.morphMax : open / std::max(cfg.jawUnits, 0.1f) * 100.0f;
				want[1] = cfg.megaMouth ? cfg.morphMax : open * 0.5f / std::max(cfg.lipUnits, 0.1f) * 100.0f;
				want[2] = cfg.megaMouth ? cfg.morphMax : std::max(0.0f, (width + cfg.mouthExtra - restWidth) * 0.5f) / std::max(cfg.widenUnits, 0.1f) * 100.0f;
			}
			if (cfg.throatMorphs && (through || inMouth)) {
				auto ramp = [&](float a, float b) { return std::clamp((depth - a) / (b - a), 0.0f, 1.0f) * cfg.morphMax; };
				want[3] = ramp(5.0f, 8.5f);
				want[4] = ramp(8.5f, 12.0f);
				want[5] = ramp(12.0f, 20.0f);
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
			if (cfg.debugLog && g_logTimer > 1.0f)
				logs::info("DeepFit: mouth - shaft {:.2f} tall, {:.2f} wide at the lips, tip {:.1f} deep | jaw {:.0f} round {:.0f} widen {:.0f} throat {:.0f}/{:.0f}/{:.0f}{}",
					height, width, depth, r.morph[0], r.morph[1], r.morph[2], r.morph[3], r.morph[4], r.morph[5], cfg.megaMouth ? " (mega mouth)" : "");
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
				if (cfg.mouthMorphs) {   // 0.2.0: the morphs opened the mouth; the bones fit the lips to the shaft (out, or back in round a slim one)
					target[i] = sMax > -1e8f ? std::clamp(sMax + cfg.lipMargin + cfg.stretch - sRest - comp[i], -cfg.lipCloseMax, mx) : 0.0f;
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

	// 0.2.0: hand the mouth morphs to MFEE (its Papyrus function, on the game thread we run on) and the teeth modifier
	// to the face; only when a value has really changed, at most 20 times a second
	void sendMouth(Receiver& r, float delta)
	{
		auto& cfg = Config::get();
		if (!cfg.mouthMorphs)
			return;
		auto act = r.actor.get();
		if (!act)
			return;
		if (auto* fg = act->GetFaceGenAnimationData(); fg && cfg.teethOpen)
			fg->modifierKeyFrame.SetValue(12, r.teeth ? 1.0f : 0.0f);
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
		BSEventNotifyControl ProcessEvent(InputEvent* const*, BSTEventSource<InputEvent*>*) override
		{
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

void DeepFit::Install()
{
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
