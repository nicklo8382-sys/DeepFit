#pragma once

namespace DeepFit
{
	void Install();            // frame driver (at input loaded)
	void OnFrame(float delta); // called once per frame from the frame driver
}
