#ifndef JSK_RVIZ_PLUGINS_IMAGE_TRANSPORT_HINTS_PROPERTY_H
#define JSK_RVIZ_PLUGINS_IMAGE_TRANSPORT_HINTS_PROPERTY_H

#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/editable_enum_property.hpp>
#include <image_transport/transport_hints.hpp>

namespace jsk_rviz_plugins {

class ImageTransportHintsProperty : public rviz::EditableEnumProperty
{
  Q_OBJECT
 public:
  ImageTransportHintsProperty(const char* name, const char* description,
                              rviz::Property* parent, const char* changed_slot);
  ~ImageTransportHintsProperty();

  image_transport::TransportHints getTransportHints();
};
}
#endif
