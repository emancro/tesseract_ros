/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef TESSERACT_RVIZ_MARKERS_ARROW_MARKER_H
#define TESSERACT_RVIZ_MARKERS_ARROW_MARKER_H

#ifndef Q_MOC_RUN
#include <tesseract_rviz/markers/marker_base.h>
#include <OgreVector3.h>
#endif

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class Arrow;
class DisplayContext;
}

namespace tesseract_rviz
{

class ArrowMarker: public MarkerBase
{
public:
  using Ptr = boost::shared_ptr<ArrowMarker>;
  using ConstPtr = boost::shared_ptr<const ArrowMarker>;

  ArrowMarker(const std::string &ns,
              const int id,
              rviz::DisplayContext* context,
              Ogre::SceneNode* parent_node);

  ArrowMarker(const std::string &ns,
              const int id,
              Ogre::Vector3 point1,
              Ogre::Vector3 point2,
              Ogre::Vector3 scale,
              rviz::DisplayContext* context,
              Ogre::SceneNode* parent_node );

  ~ArrowMarker() override;

  void setScale(Ogre::Vector3 scale) override;
  Ogre::Vector3 getScale() const override;

  void setColor( float r, float g, float b, float a ) override;

  std::set<Ogre::MaterialPtr> getMaterials() override;

protected:
  virtual void setDefaultProportions();

  rviz::Arrow *arrow_;
  Ogre::SceneNode *child_scene_node_;
  Ogre::Vector3 scale_;
};

}
#endif // TESSERACT_RVIZ_MARKERS_ARROW_MARKER_H