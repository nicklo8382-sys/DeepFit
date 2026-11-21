# DeepFit

SKSE plugin that drives the [SMP Head - Male HPH] mouth bones from code instead of physics.
Every frame it skins the nearby partner's schlong mesh on the CPU (so TNG scale/bend, OBody
morphs and the real mesh shape - bumps, knots, flares - are all respected) and pushes the
lip / corner / nose-base / throat bones straight out to the shaft surface.

* No springs, no jitter, no flapping outside scenes (bones sit at rest unless a shaft is there).
* Throat bulge: the three Throat Center bones are pushed forward by whatever part of the shaft
  is level with them, so it only appears on deep insertion and follows the shaft's shape.
* Works for the player and any NPC whose head has the SMP Head mouth bones.

## Build (GitHub Actions, no local tools needed)
1. Create a new GitHub repository and upload everything in this folder
   (`xmake.lua`, `src/`, `mod/`, `.github/`).
2. Open the **Actions** tab, enable workflows, and the push builds automatically
   (~10-15 min: it downloads and compiles CommonLibSSE-NG the first time).
3. Download the **DeepFit** artifact: it contains `SKSE/Plugins/DeepFit.dll`,
   `.pdb` and `.ini`.

## Install
Put the artifact's `SKSE` folder and the `mod/` folder's files into a mod placed
**below** "SMP Head - Male HPH". The included `HP_Head_Male.xml` hands the lips,
nose base and throat over to the DLL (only the cheeks stay on SMP).
FSMP must stay installed (it merges the head bones into the skeleton), but the
"Facial Morph Collision" DLL is no longer needed.

## Tuning
`SKSE/Plugins/DeepFit.ini` - see the comments. `DebugLog = 1` writes the bone
pushes once a second to `My Games/Skyrim Special Edition/SKSE/DeepFit.log`.

Mouth (0.2.0)
  DeepFit is the one thing that moves the mouth of an SMP Head actor, so other mods can build on it:
  it opens the jaw and rounds / widens the lips with the head's own MFEE morphs (PPA_JawWide,
  PPA_LipStretch, PPA_LipWiden) to the partner's shaft thickness where it passes the lips - a slim
  shaft gets a small O, a thick one or a knot a wide one, and it closes again as it slides out -
  then fits the lip bones snugly round it. PPA_TH1-3 swell the throat by depth, and the teeth part.
  MegaMouth = 1 opens it as wide as it goes instead. Settings: DeepFit.ini, section "the mouth itself".
  The bundled PPA preset (ppa-override-configs\00SMP_Head_PPA_Male.toml) only quiets OStim's
  expression while the mouth is in use. Set MouthMorphs = 0 to give the morphs back to PPA or another mod.

In game (0.3.0)
  With SKSE Menu Framework installed, DeepFit has its own page (DeepFit > Mouth): every setting, applied at
  once and saved to DeepFit.ini. "Mega mouth key" sets a key that switches Mega mouth on and off in a scene.
  Other mods: SendModEvent("DeepFit_MegaMouth", "", 1.0) turns Mega mouth on, 0.0 off, -1.0 back to the
  setting; "DeepFit_Mouth" the same for DeepFit opening the mouth at all.
