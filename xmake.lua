-- DeepFit: code-driven SMP-Head lip / nose / throat bones for oral scenes
set_xmakever("2.8.2")

set_project("DeepFit")
set_version("0.1.1")
set_license("GPL-3.0")
set_languages("c++23")
set_warnings("allextra")
set_encodings("utf-8")

add_rules("mode.debug", "mode.releasedbg")

-- CommonLibSSE-NG from xrepo (no submodules needed). SE + AE only.
add_requires("commonlibsse-ng", { configs = { skyrim_vr = false } })

target("DeepFit")
    add_packages("commonlibsse-ng")
    add_rules("@commonlibsse-ng/plugin", {
        name = "DeepFit",
        author = "Nick",
        description = "Drives the SMP Head lip/nose/throat bones from the partner's schlong mesh (no physics)"
    })
    add_files("src/**.cpp")
    add_headerfiles("src/**.h")
    add_includedirs("src")
    set_pcxxheader("src/pch.h")
