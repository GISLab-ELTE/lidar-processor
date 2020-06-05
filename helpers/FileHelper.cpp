/*
 * BSD 3-Clause License
 * Copyright (c) 2020, Roxána Provender
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#include <string>

#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>

#include "FileHelper.h"

namespace olp
{
namespace helper
{
void createDirectory(const std::string& path)
{
    boost::filesystem::path dir(path);

    if (!(boost::filesystem::exists(dir)))
    {
        boost::filesystem::create_directory(dir);
    }
}

std::vector<std::string> getFilenames(const boost::filesystem::path& path)
{
    std::vector<std::string> filenames;
    const boost::filesystem::directory_iterator end{};

    for (boost::filesystem::directory_iterator iter{path}; iter != end; ++iter)
    {
        if (boost::filesystem::is_regular_file(*iter))
            filenames.push_back(iter->path().string());
    }

    return filenames;
}
} // file
} // olp
