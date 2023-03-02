// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SIMPLE_IMAGE_RECON__FRAME_HANDLER_HPP_
#define SIMPLE_IMAGE_RECON__FRAME_HANDLER_HPP_

namespace simple_image_recon
{
template <typename ImageConstSharedPtrT>
class FrameHandler
{
public:
  virtual void frame(
    const ImageConstSharedPtrT & img, const std::string & topic) = 0;
  virtual ~FrameHandler() {}
};
}  // namespace simple_image_recon
#endif  // SIMPLE_IMAGE_RECON__FRAME_HANDLER_HPP_
