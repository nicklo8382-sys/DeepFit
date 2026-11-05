#include "Config.h"
#include "DeepFit.h"
#include <spdlog/sinks/basic_file_sink.h>

namespace
{
	void SetupLog()
	{
		auto dir = SKSE::log::log_directory();
		if (!dir)
			return;
		auto path = *dir / "DeepFit.log";
		auto sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(path.string(), true);
		auto log = std::make_shared<spdlog::logger>("global", std::move(sink));
		log->set_level(spdlog::level::info);
		log->flush_on(spdlog::level::info);
		spdlog::set_default_logger(std::move(log));
		spdlog::set_pattern("[%H:%M:%S.%e] [%l] %v");
	}

	void MessageHandler(SKSE::MessagingInterface::Message* a_msg)
	{
		if (a_msg && a_msg->type == SKSE::MessagingInterface::kDataLoaded) {
			Config::get().load();
		}
	}
}

SKSEPluginLoad(const SKSE::LoadInterface* a_skse)
{
	SKSE::Init(a_skse);
	SetupLog();
	logs::info("DeepFit 0.1.1 loading");
	Config::get().load();
	if (auto* msg = SKSE::GetMessagingInterface())
		msg->RegisterListener(MessageHandler);
	DeepFit::Install();
	return true;
}
