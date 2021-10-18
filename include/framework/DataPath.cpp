// This code contains NVIDIA Confidential Information and is disclosed
// under the Mutual Non-Disclosure Agreement.
//
// Notice
// ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS" NVIDIA MAKES
// NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
//
// NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. No third party distribution is allowed unless
// expressly authorized by NVIDIA.  Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2020 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#include "DataPath.hpp"

#include <stdexcept>
#include <numeric>
#include <array>
#include <iostream>

#include <cstdlib>
#include <climits>

#include <unistd.h>
#include <sys/stat.h>

namespace dw_samples
{
namespace common
{

std::vector<std::string> DataPath::getSearchPaths()
{
    // Collect list of search-paths
    auto searchPaths = std::vector<std::string>{};

    // Prefer env variable "SDK_TESTS_DATA_PATH" over other search paths
    {
        auto const envDataPath = std::getenv("SDK_TESTS_DATA_PATH");
        if (envDataPath)
        {
            searchPaths.emplace_back(envDataPath);
        }
    }

    // Tokenize the current executables path, and search upwards for a 'data' folder (living in root of install tree / bazel sandbox).
    // Also look for '_data', as the cmake build folder is configured to have a '_data' symlink in it's root (allowing in-build folder execution)
    {
        auto path = Executable::getFolderPath();
        if (path.size() > 0U)
        {
            while (path.size() > 1u)
            {
                searchPaths.emplace_back(path + "/_data");     // corresponds to cmake in-build folder symlink name
                searchPaths.emplace_back(path + "/data");      // corresponds to install folder / bazel sandbox name
                path = path.substr(0, path.find_last_of("/")); // peel of one folder at a time from full path
            }
        }
        else
        {
            // Fallback to use simple well-known relative paths if the executable's path can't be determined
            auto relativePath            = std::string{"data"};
            constexpr uint32_t MAX_DEPTH = 5U;
            for (auto depth = 0U; depth < MAX_DEPTH; ++depth)
            {
                relativePath.insert(0, "../");
                searchPaths.push_back(relativePath);
            }
        }
    }

    return searchPaths;
}

std::string DataPath::getBasePathFor(std::string const& fileOrDirectory,
                                     std::vector<std::string> const& searchPaths)
{
    for (auto const& searchPath : searchPaths)
    {
        auto const fullPath = searchPath + "/" + fileOrDirectory;
        if (Filesystem::fileExists(fullPath.c_str()) || Filesystem::directoryExists(fullPath.c_str()))
        {
            return searchPath;
        }
    }

    // File was not found in search paths - print warning and return empty string
    {
        auto warningMessage = std::string{"Warning: DataPath::getBasePathFor(): '"} + fileOrDirectory + "' not found in search paths [";
        auto searchPathsString =
            std::accumulate(std::begin(searchPaths), std::end(searchPaths), std::string{},
                            [](std::string const& ss, std::string const& s) {
                                return ss.empty() ? s : ss + " | " + s;
                            });
        std::cerr << warningMessage + searchPathsString + "]" << std::endl;
        return "";
    }
}

std::string DataPath::getPathFor(std::string const& fileOrDirectory,
                                 std::vector<std::string> const& searchPaths)
{
    return getBasePathFor(fileOrDirectory, searchPaths) + "/" + fileOrDirectory;
}

std::string DataPath::getBasePath(std::vector<std::string> const& searchPaths)
{
    return getBasePathFor("DATA_ROOT", searchPaths);
}

std::string DataPath::get()
{
    return getBasePath();
}

char const* DataPath::get_cstr()
{
    static auto BASE_PATH = std::string{};
    BASE_PATH             = getBasePath();
    return BASE_PATH.c_str();
}

std::string Executable::getPath()
{
    auto res = std::array<char, PATH_MAX>{};
    if (!::readlink("/proc/self/exe", res.data(), PATH_MAX))
    {
        throw std::runtime_error("Executable::getPath() - no absolute executable path found");
    }
    return {res.data()};
}

std::string Executable::getFolderPath()
{
    auto const exePath = getPath();
    return exePath.substr(0, exePath.find_last_of("/"));
}

bool Filesystem::fileExists(char const* fileName)
{
    struct stat file
    {
    };
    return (::stat(fileName, &file) == 0 && S_ISREG(file.st_mode));
}

bool Filesystem::directoryExists(char const* directoryName)
{
    struct stat sb
    {
    };
    return (::stat(directoryName, &sb) == 0 && S_ISDIR(sb.st_mode));
}

} // namespace common
} // namespace dw_tools