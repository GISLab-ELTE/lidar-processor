/*
 * BSD 3-Clause License
 * Copyright (c) 2020, Roxána Provender
 * All rights reserved.
 *
 * You may obtain a copy of the License at
 * https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef OLP_FILEHELPER_H
#define OLP_FILEHELPER_H

namespace olp
{
namespace helper
{
std::vector<std::string> getFilenames(const boost::filesystem::path& path);

void createDirectory(const std::string& path);
} // file
} // olp

#endif //OLP_FILEHELPER_H
