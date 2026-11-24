#pragma once
#include "Config.h"

// 0.4.6: head profiles - one file per head in Data\SKSE\Plugins\DeepFit\Heads\*.ini. Each says which races (or head
// meshes) it is for, which bones its head has and which way each one pushes, and keeps its own copy of the per-head
// settings (the same ones the women's set keeps). A new head - another race, a custom head - is a new file; nothing
// needs compiling.
struct HeadBone
{
	std::string name;
	std::string kind;   // Lip, Corner, Nose, Throat, Cheek
	float       x = 0.0f, y = 0.0f, z = 0.0f;
};

struct HeadProfile
{
	std::string              name;         // shown in the menu
	std::string              file;         // the .ini it came from (its settings are saved back there)
	std::vector<std::string> races;        // race editor IDs, lower case ("*" = every race)
	std::vector<std::string> meshes;       // head mesh file names, lower case ("maleheadargonian.nif")
	int                      sex = 0;      // 0 men, 1 women, -1 both
	std::vector<HeadBone>    bones;        // bones it adds or changes (the rest keep DeepFit's own list)
	float                    jawDrop = 4.1f;    // 0.4.6: how far his lower lip drops with the jaw fully open (units; the test centres in it)
	int                      jawPhoneme = -1;   // 0.4.5: a head without DeepFit's morphs opens its jaw with this phoneme (BigAah = 1)
	Config                   cfg;          // its per-head settings (shared settings come from the main set)
};

namespace Heads
{
	void                      Load();                      // reads every profile (the main set must be loaded first)
	std::vector<HeadProfile>& All();
	int                       Generation();                // goes up on every Load (receivers then look again)
	// the profile for this head: its race's editor ID, its face mesh's file path, and whether it is a woman's (-1 = none)
	int                       Find(const std::string& raceEditorID, const std::string& headMesh, bool female);
	void                      SyncAll();                   // the shared settings changed: every profile takes them
	const char*               Dir();
}
