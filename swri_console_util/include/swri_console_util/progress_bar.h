// *****************************************************************************
//
// Copyright (c) 2014, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#ifndef CONSOLE_UTIL_PROGRESS_BAR_H_
#define CONSOLE_UTIL_PROGRESS_BAR_H_

#include <termios.h>
#include <string>

#include <ros/ros.h>

namespace swri_console_util
{
  class ProgressBar
  {
  public:
    ProgressBar();
    ~ProgressBar();

    void SetStartTime(const ros::WallTime& start_time);
    void SetProgress(double percent_complete);
    void PrintTime();
    void CheckForPause();
    signed char ReadCharFromStdin();

    static std::string GetTimeString(double seconds);
    static std::string IntToString(int64_t i, int width = 0);

  private:
    void SetupTerminal();
    void RestoreTerminal();

    bool paused_;

    double percent_complete_;

    ros::WallTime start_time_;
    ros::WallDuration paused_time_;

    termios orig_flags_;
    fd_set  stdin_fdset_;
    int     maxfd_;
  };
}

#endif  // CONSOLE_UTIL_PROGRESS_BAR_H_
