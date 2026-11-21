#ifndef APP_TARGET_H
#define APP_TARGET_H

#include <memory>
#include <string_view>
#include "../common/target.h"

class Target {
private:
	TargetDefinition const &definition_;
	std::string_view name_;
public:
	Target(TargetDefinition const &definition, std::string_view name)
		: definition_(definition)
		, name_(name)
	{
	}

	TargetDefinition const &definition() const
	{
		return definition_;
	}

	std::string_view name() const
	{
		return name_;
	}
	
	bool is_signature(uint32_t sig) const
	{
		return definition_.matches_signature((sig >> 16) & 0xff, (sig >> 8) & 0xff, sig & 0xff);
	}
	
	int32_t sigrow_address() const
	{
		if (definition_.supports(CapabilityUpdi)) {
			return definition_.updi.sigrow_address;
		}
		return -1;
	}	
};

inline std::unique_ptr<Target> create_target(std::string_view name)
{
	if (name == "atmega8") {
		return std::make_unique<Target>(TargetATmega8, "atmega8");
	}
	if (name == "atmega48") {
		return std::make_unique<Target>(TargetATmega48, "atmega48");
	}
	if (name == "atmega88") {
		return std::make_unique<Target>(TargetATmega88, "atmega88");
	}
	if (name == "atmega168") {
		return std::make_unique<Target>(TargetATmega168, "atmega168");
	}
	if (name == "atmega328p") {
		return std::make_unique<Target>(TargetATmega328P, "atmega328p");
	}
	if (name == "attiny2313") {
		return std::make_unique<Target>(TargetATtiny2313, "attiny2313");
	}
	if (name == "attiny13a") {
		return std::make_unique<Target>(TargetATtiny13A, "attiny13a");
	}
	if (name == "attiny202") {
		return std::make_unique<Target>(TargetATtiny202, "attiny202");
	}
	if (name == "avr64dd28") {
		return std::make_unique<Target>(TargetAVR64DD28, "avr64dd28");
	}
	if (name == "avr64dd32") {
		return std::make_unique<Target>(TargetAVR64DD32, "avr64dd32");
	}
	if (name == "avr64du32") {
		return std::make_unique<Target>(TargetAVR64DU32, "avr64du32");
	}
	return nullptr;
}

#endif // APP_TARGET_H
