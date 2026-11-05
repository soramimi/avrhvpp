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
	return nullptr;
}

#endif // APP_TARGET_H
