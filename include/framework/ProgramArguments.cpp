/////////////////////////////////////////////////////////////////////////////////////////
// This code contains NVIDIA Confidential Information and is disclosed
// under the Mutual Non-Disclosure Agreement.

// Notice
// ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS" NVIDIA MAKES
// NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.

// NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. No third party distribution is allowed unless
// expressly authorized by NVIDIA.  Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.

// Copyright (c) 2014-2020 NVIDIA Corporation. All rights reserved.

// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.

/////////////////////////////////////////////////////////////////////////////////////////

#include "ProgramArguments.hpp"

#include <sstream>
#include <string>
#include <string.h>
#include <iostream>
#include <algorithm>

ProgramArguments::ProgramArguments(bool addDefaults /* = true */)
{
    if (addDefaults)
    {
        arguments.insert({"profiling", Option_t("profiling", "1", "Enables/disables sample profiling")});
        arguments.insert({"offscreen", Option_t("offscreen", "0", "Used for running windowed apps in headless mode. 0 = show window, 1 = offscreen window, 2 = no window created")});
    }
}

ProgramArguments::ProgramArguments(const std::vector<Option_t>& options, bool addDefaults /* = true */)
    : ProgramArguments(addDefaults)
{
    for (auto& o : options)
    {
        addOption(o);
    }
}

ProgramArguments::ProgramArguments(int argc, const char** argv, const std::vector<Option_t>& options, const char* description)
    : ProgramArguments(options)
{
    if (description)
        setDescription(description);

    if (!parse(argc, argv))
    {
        exit(0);
    }
}

ProgramArguments::~ProgramArguments()
{
}

bool ProgramArguments::parse(const int argc, const char** argv)
{
    bool show_help = false;
    std::string unknown_options;
    // vecArgNames will check for duplicate args in the command line
    std::vector<std::string> vecArgNames;

    for (int i = 1; i < argc; ++i)
    {
        std::string arg   = argv[i];
        std::size_t mnPos = arg.find("--");

        // starts with --
        if (mnPos == 0)
        {
            arg = arg.substr(2);

            std::string name;
            std::string value;

            std::size_t eqPos = arg.find_first_of("=");

            name  = arg.substr(0, eqPos);
            value = arg.substr(eqPos + 1);

            // support space-separated args
            if (eqPos == std::string::npos && i + 1 < argc && argv[i + 1][0] != '-')
            {
                value = argv[++i];
            }
            if (std::find(vecArgNames.begin(), vecArgNames.end(), name) != vecArgNames.end())
            {
                // this option has already been parsed
                throw std::runtime_error("Duplicate inputs found. Option --" + name + " specified more than once. ");
            }
            vecArgNames.push_back(name);

            if (name == "help")
            {
                show_help = true;
                continue;
            }

            auto option_it = arguments.find(name);
            if (option_it == arguments.end())
            {
                unknown_options += "--" + name + " ";
                show_help = true;
            }
            else
            {
                option_it->second.parsed = true;
                option_it->second.value  = value;
            }
        }
    }

    // Check Required Arguments
    std::vector<std::string> missing_required;
    for (auto& option : arguments)
    {
        if (option.second.required && option.second.value.empty())
        {
            missing_required.push_back(option.second.option);
        }
    }
    if (!missing_required.empty())
    {
        std::string missing_required_message;
        std::string example_usage;
        for (std::string required_argument : missing_required)
        {
            missing_required_message.append("\"");
            missing_required_message.append(required_argument);
            missing_required_message.append("\", ");

            example_usage.append(" --");
            example_usage.append(required_argument);
            example_usage.append("=<value>");
        }
        std::string executable = argv[0];

        std::cout << "ProgramArguments: Missing required arguments: "
                  << missing_required_message
                  << "e.g.\n\t" << executable << example_usage
                  << "\n";

        show_help = true;
    }

    // Show Help?
    if (show_help)
    {
        printHelp();
        if (unknown_options.size() > 0)
        {
            throw std::runtime_error("Unknown option " + unknown_options + "\n");
        }
        return false;
    }

    return true;
}

const std::string& ProgramArguments::get(const char* name) const
{
    auto it = arguments.find(name);
    if (it == arguments.end())
    {
        printf("ProgramArguments: Missing argument '%s' requested\n", name);
        return m_empty;
    }
    else
        return it->second.value;
}

bool ProgramArguments::has(const char* name) const
{
    auto it = arguments.find(name);
    if (it == arguments.end())
        return false;

    return !it->second.value.empty();
}

bool ProgramArguments::wasSpecified(const char* name) const
{
    auto it = arguments.find(name);
    if (it == arguments.end())
        return false;

    return it->second.parsed;
}

bool ProgramArguments::enabled(const char* name) const
{
    if (!has(name))
        return false;

    auto value = get(name);
    if (value == "true")
        return true;
    if (value == "false")
        return false;

    // Convert to int
    int valueInt = std::stoi(value, nullptr);
    return valueInt != 0;
}

void ProgramArguments::addOption(const Option_t& newOption)
{
    auto it = arguments.insert({newOption.option, newOption});
    if (!it.second)
        throw std::runtime_error(std::string("ProgramArguments already contains the new option: ") + newOption.option);
}

void ProgramArguments::addOptions(const std::vector<Option_t>& newOptions)
{
    for (auto const& o : newOptions)
    {
        addOption(o);
    }
}

void ProgramArguments::clearOption(const char* name)
{
    auto it = arguments.find(name);
    if (it != arguments.end())
        arguments.erase(name);
}

void ProgramArguments::clearOptions()
{
    arguments.clear();
}

void ProgramArguments::set(const char* option, const char* value)
{
    auto it = arguments.find(option);
    if (it == arguments.end())
        throw std::runtime_error(std::string("ProgramArguments: tried to set an option that doesn't exist. ") + option);
    it->second.value = value;
}

void ProgramArguments::setDefault(const char* option, const char* value)
{
    auto it = arguments.find(option);
    if (it == arguments.end())
        throw std::runtime_error(std::string("ProgramArguments: tried to set an option that doesn't exist. ") + option);
    it->second.default_value = value;
}

void ProgramArguments::setDescription(const char* description)
{
    m_description = description;
}

void ProgramArguments::printHelp() const
{
    if (arguments.empty())
    {
        std::cout << "Run application without command line arguments.\n";
        return;
    }

    if (!m_description.empty())
        std::cout << m_description << std::endl;

    std::stringstream ss;

    for (auto& arg : arguments)
    {
        auto& option = arg.second;
        ss << "--" << option.option << ": ";
        if (option.required)
            ss << "required, ";
        ss << "default=" << option.default_value;
        if (!option.help.empty())
            ss << "\n    " << option.help;
        ss << "\n";
    }

    std::cout << ss.str();
}

std::string ProgramArguments::printList() const
{
    std::stringstream ss;

    for (auto& arg : arguments)
    {
        auto& option = arg.second;
        ss << "--" << option.option << "=" << option.value << "\n";
    }

    return ss.str();
}

std::string ProgramArguments::parameterString() const
{
    std::stringstream list;

    bool first = true;
    for (auto arg : arguments)
    {
        if (!first)
            list << ",";
        list << arg.first << "=" << arg.second.value;
        first = false;
    }

    return list.str();
}
