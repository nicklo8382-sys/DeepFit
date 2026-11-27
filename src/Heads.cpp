#include "Heads.h"
#include <filesystem>
#include <mutex>

namespace
{
	std::vector<HeadProfile> g_heads;
	int                      g_gen = 0;

	std::string trim(const std::string& s)
	{
		auto a = s.find_first_not_of(" \t\r\n");
		auto b = s.find_last_not_of(" \t\r\n");
		return a == std::string::npos ? std::string() : s.substr(a, b - a + 1);
	}
	std::string lower(std::string s)
	{
		for (auto& c : s)
			c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
		return s;
	}
	std::vector<std::string> list(const std::string& v)
	{
		std::vector<std::string> out;
		std::stringstream        ss(v);
		std::string              item;
		while (std::getline(ss, item, ',')) {
			item = lower(trim(item));
			if (!item.empty())
				out.push_back(item);
		}
		return out;
	}
	std::string fileName(std::string p)
	{
		for (auto& c : p)
			if (c == '/')
				c = '\\';
		auto s = p.find_last_of('\\');
		return lower(s == std::string::npos ? p : p.substr(s + 1));
	}

	// [Head] Name / Races / HeadMeshes / Sex, [Bones] "Bone Name = Kind, x, y, z"; the settings (any section, usually
	// [Settings]) are read by Config::loadFile
	bool readProfile(const std::filesystem::path& path, HeadProfile& h)
	{
		std::ifstream f(path);
		if (!f)
			return false;
		h.file = path.string();
		h.name = path.stem().string();
		std::string line, section;
		while (std::getline(f, line)) {
			auto hash = line.find_first_of(";#");
			if (hash != std::string::npos)
				line = line.substr(0, hash);
			line = trim(line);
			if (line.empty())
				continue;
			if (line.front() == '[' && line.back() == ']') {
				section = lower(trim(line.substr(1, line.size() - 2)));
				continue;
			}
			auto eq = line.find('=');
			if (eq == std::string::npos)
				continue;
			const std::string k = trim(line.substr(0, eq)), v = trim(line.substr(eq + 1));
			if (section == "head") {
				const std::string lk = lower(k);
				if (lk == "name")
					h.name = v;
				else if (lk == "races")
					h.races = list(v);
				else if (lk == "headmeshes")
					h.meshes = list(v);
				else if (lk == "jawdrop") {   // 0.4.6
					try { h.jawDrop = std::stof(v); } catch (...) {}
				} else if (lk == "jawphoneme") {   // 0.4.5
					static const char* kPh[] = { "aah", "bigaah", "bmp", "chjsh", "dst", "eee", "eh", "fv", "i", "k", "n", "oh", "oohq", "r", "th", "w" };
					const std::string ph = lower(v);
					for (int i = 0; i < 16; ++i)
						if (ph == kPh[i])
							h.jawPhoneme = i;
				} else if (lk == "jawbone") {   // 0.5.5
					h.jawBone = v;
				} else if (lk == "jawbonedeg") {
					try { h.jawBoneDeg = std::stof(v); } catch (...) {}
				} else if (lk == "upperlipat" || lk == "lowerlipat") {
					float*            dst = lk == "upperlipat" ? h.upperLipAt : h.lowerLipAt;
					std::stringstream ss(v);
					std::string       part;
					int               n = 0;
					while (n < 3 && std::getline(ss, part, ','))
						try { dst[n++] = std::stof(trim(part)); } catch (...) {}
					if (n == 3 && lk == "lowerlipat")
						h.hasLips = true;
				} else if (lk == "hidden") {   // 0.5.7
					const std::string s = lower(v);
					h.hidden = s == "1" || s == "true" || s == "yes";
				} else if (lk == "sex") {
					const std::string s = lower(v);
					h.sex = s == "female" ? 1 : (s == "any" || s == "both") ? -1 : 0;
				}
			} else if (section == "bones") {
				HeadBone b;
				b.name = k;
				std::stringstream ss(v);
				std::string       part;
				std::vector<std::string> parts;
				while (std::getline(ss, part, ','))
					parts.push_back(trim(part));
				if (parts.size() < 4)
					continue;
				b.kind = parts[0];
				try {
					b.x = std::stof(parts[1]);
					b.y = std::stof(parts[2]);
					b.z = std::stof(parts[3]);
				} catch (...) {
					continue;
				}
				h.bones.push_back(b);
			}
		}
		return true;
	}
}

namespace Heads
{
	const char* Dir() { return "Data/SKSE/Plugins/DeepFit/Heads"; }

	std::vector<HeadProfile>& All() { return g_heads; }
	int                       Generation() { return g_gen; }

	void Load()
	{
		std::vector<HeadProfile> heads;
		std::error_code          ec;
		if (std::filesystem::is_directory(Dir(), ec)) {
			std::vector<std::filesystem::path> files;
			for (auto& e : std::filesystem::directory_iterator(Dir(), ec))
				if (e.is_regular_file() && lower(e.path().extension().string()) == ".ini")
					files.push_back(e.path());
			std::sort(files.begin(), files.end());
			for (auto& p : files) {
				HeadProfile h;
				if (!readProfile(p, h))
					continue;
				// its settings: the main set (or the women's, for a woman's head), then its own per-head ones over it
				h.cfg = h.sex == 1 ? Config::female() : Config::get();
				h.cfg.isFemale = false;
				h.cfg.ownFile = h.file;
				h.cfg.loadFile(h.file, true);
				logs::info("DeepFit: head profile \"{}\" ({} races, {} head meshes, {} bones) from {}", h.name, h.races.size(), h.meshes.size(), h.bones.size(),
					p.filename().string());
				heads.push_back(std::move(h));
			}
		}
		g_heads = std::move(heads);
		++g_gen;
	}

	int Find(const std::string& raceEditorID, const std::string& headMesh, bool female)
	{
		const std::string race = lower(raceEditorID), mesh = fileName(headMesh);
		int               byRace = -1;
		for (int i = 0; i < static_cast<int>(g_heads.size()); ++i) {
			const auto& h = g_heads[i];
			if (h.sex != -1 && h.sex != (female ? 1 : 0))
				continue;
			// a head mesh named in the profile wins (custom races that use that head), then the race
			if (!mesh.empty() && std::find(h.meshes.begin(), h.meshes.end(), mesh) != h.meshes.end())
				return i;
			if (byRace < 0 && !race.empty() && (std::find(h.races.begin(), h.races.end(), race) != h.races.end() ||
			                                       std::find(h.races.begin(), h.races.end(), "*") != h.races.end()))
				byRace = i;
		}
		return byRace;
	}

	void SyncAll()
	{
		for (auto& h : g_heads) {
			const auto mine = h.cfg.keyValues(true);
			const std::string file = h.cfg.ownFile;
			h.cfg = h.sex == 1 ? Config::female() : Config::get();
			h.cfg.isFemale = false;
			h.cfg.ownFile = file;
			for (auto& [k, v] : mine)
				h.cfg.applyKey(k, v);
		}
	}
}
